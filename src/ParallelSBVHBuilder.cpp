#include "ParallelSBVHBuilder.hpp"

#include <iterator>
#include <pdqsort.h>
#include <queue>
#include <random>
#include <spdlog/spdlog.h>

void ParallelSBVHBuilder::Run() {
	if (kThreadCount > 1)
		m_thread_group = std::make_unique<ThreadUnit[]>(kThreadCount - 1);
	m_consumer_tokens.reserve(kThreadCount);
	m_producer_tokens.reserve(kThreadCount);
	m_thread_node_allocators.reserve(kThreadCount);
	m_thread_reference_allocators.reserve(kThreadCount);

	m_thread_tmp_references.resize(kThreadCount);
	for (uint32_t i = 0; i < kThreadCount; ++i) {
		m_consumer_tokens.emplace_back(m_task_queue);
		m_producer_tokens.emplace_back(m_task_queue);
		m_thread_node_allocators.emplace_back(m_bvh.m_node_pool);
		m_thread_reference_allocators.emplace_back(m_reference_pool);
	}

	spdlog::info("Begin, threshold = {}", kLocalRunThreshold);
	auto begin = std::chrono::steady_clock::now();
	make_root_task().BlockRun();
	spdlog::info(
	    "End {} ms",
	    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count());

	m_bvh.m_leaf_cnt = m_leaf_count;
}

ParallelSBVHBuilder::Task ParallelSBVHBuilder::make_root_task() {
	uint32_t root_idx = m_thread_node_allocators[0].Alloc();
	assert(root_idx == 0);
	m_node_pool[root_idx].aabb = m_scene.GetAABB();
	std::vector<uint32_t> references;
	{
		references.reserve(m_scene.GetTriangles().size());
		for (uint32_t i = 0; i < m_scene.GetTriangles().size(); ++i) {
			uint32_t ref_idx = m_thread_reference_allocators[0].Alloc();
			auto &ref = m_reference_pool[ref_idx];
			ref.tri_idx = i;
			ref.aabb = m_scene.GetTriangles()[i].GetAABB();
			references.push_back(ref_idx);
		}
	}
	return Task{this, root_idx, std::move(references), 0, 0, kThreadCount};
}

std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task> ParallelSBVHBuilder::Task::Run() {
	if (m_references.size() == 1) {
		make_leaf();
		return {};
	}

	ObjectSplit object_split = find_object_split();
	SpatialSplit spatial_split{};
	if (m_depth <= m_p_builder->m_config.m_max_spatial_depth) {
		AABB overlap = object_split.left_aabb;
		overlap.IntersectAABB(object_split.right_aabb);
		if (overlap.GetHalfArea() >= m_p_builder->m_min_overlap_area)
			spatial_split = find_spatial_split();
	}
	if (spatial_split.sah < object_split.sah) {
		auto ret = perform_spatial_split(spatial_split);
		if (!PairEmpty(ret))
			return ret;
	}
	return perform_object_split(object_split);
}

void ParallelSBVHBuilder::Task::BlockRun() {
	auto [left_task, right_task] = Run();
	if (m_thread_count > 1) {
		if (left_task.Empty()) // Leaf
			return;
		if (left_task.m_thread_count == 0) {
			++m_p_builder->m_task_count;
			m_p_builder->m_task_queue.enqueue(get_queue_producer_token(), std::move(left_task));
			right_task.BlockRun();
		} else if (right_task.m_thread_count == 0) {
			++m_p_builder->m_task_count;
			m_p_builder->m_task_queue.enqueue(get_queue_producer_token(), std::move(right_task));
			left_task.BlockRun();
		} else {
			// Subdivide the thread
			auto future = right_task.AsyncRun();
			left_task.BlockRun();
			future.wait();
		}
	} else if (m_thread_count == 1) {
		// Begin task pool
		if (!left_task.Empty()) {
			m_p_builder->m_task_count.fetch_add(2);
			m_p_builder->m_task_queue.enqueue(get_queue_producer_token(), std::move(left_task));
			m_p_builder->m_task_queue.enqueue(get_queue_producer_token(), std::move(right_task));
		}

		Task task;
		while (m_p_builder->m_task_count.load()) {
			if (m_p_builder->m_task_queue.try_dequeue(get_queue_consumer_token(), task)) {
				task.assign_to_thread(m_thread);

				if (task.m_references.size() <= kLocalRunThreshold) {
					task.LocalRun();
					--m_p_builder->m_task_count;
				} else {
					auto new_tasks = task.Run();
					if (Task::PairEmpty(new_tasks)) {
						--m_p_builder->m_task_count;
					} else {
						++m_p_builder->m_task_count;
						m_p_builder->m_task_queue.enqueue(get_queue_producer_token(),
						                                  std::move(std::get<1>(new_tasks)));
						m_p_builder->m_task_queue.enqueue(get_queue_producer_token(),
						                                  std::move(std::get<0>(new_tasks)));
					}
				}
			}
		}
	}
}

std::future<void> ParallelSBVHBuilder::Task::AsyncRun() { return get_thread_unit(0).Push(&Task::BlockRun, this); }

void ParallelSBVHBuilder::Task::LocalRun() {
	auto new_tasks = Run();
	if (!PairEmpty(new_tasks)) {
		std::get<1>(new_tasks).LocalRun();
		std::get<0>(new_tasks).LocalRun();
	}
}

template <uint32_t DIM, typename Iter> void ParallelSBVHBuilder::sort_references(Iter first_ref, Iter last_ref) {
	pdqsort_branchless(first_ref, last_ref, [this](uint32_t l, uint32_t r) {
		return m_reference_pool[l].aabb.GetDimCenter<DIM>() < m_reference_pool[r].aabb.GetDimCenter<DIM>();
	});
}
template <typename Iter> void ParallelSBVHBuilder::sort_references(Iter first_ref, Iter last_ref, uint32_t dim) {
	dim == 0 ? sort_references<0>(first_ref, last_ref)
	         : (dim == 1 ? sort_references<1>(first_ref, last_ref) : sort_references<2>(first_ref, last_ref));
}

std::tuple<ParallelSBVHBuilder::Reference, ParallelSBVHBuilder::Reference>
ParallelSBVHBuilder::split_reference(const ParallelSBVHBuilder::Reference &ref, uint32_t dim, float pos) const {
	Reference left, right;
	left.aabb = right.aabb = AABB();
	left.tri_idx = right.tri_idx = ref.tri_idx;

	const Triangle &tri = m_scene.GetTriangles()[ref.tri_idx];
	for (uint32_t i = 0; i < 3; ++i) {
		const glm::vec3 &v0 = tri.positions[i], &v1 = tri.positions[(i + 1) % 3];
		float p0 = v0[(int)dim], p1 = v1[(int)dim];
		if (p0 <= pos)
			left.aabb.Expand(v0);
		if (p0 >= pos)
			right.aabb.Expand(v0);

		if ((p0 < pos && pos < p1) || (p1 < pos && pos < p0)) { // Edges
			glm::vec3 x = glm::mix(v0, v1, glm::clamp((pos - p0) / (p1 - p0), 0.0f, 1.0f));
			left.aabb.Expand(x);
			right.aabb.Expand(x);
		}
	}

	left.aabb.max[(int)dim] = pos;
	left.aabb.IntersectAABB(ref.aabb);

	right.aabb.min[(int)dim] = pos;
	right.aabb.IntersectAABB(ref.aabb);

	return {left, right};
}

std::tuple<uint32_t, uint32_t> ParallelSBVHBuilder::Task::get_child_thread_counts(uint32_t left_ref_count,
                                                                                  uint32_t right_ref_count) const {
	if (m_thread_count == 0)
		return {0u, 0u};
	auto lt = (float)left_ref_count, tt = float(left_ref_count + right_ref_count);
	auto lc = std::clamp(uint32_t(glm::round(lt / tt * float(m_thread_count))), 0u, m_thread_count);
	return {lc, m_thread_count - lc};
}
/*
 Spatial split
 */
template <uint32_t DIM> void ParallelSBVHBuilder::Task::_find_spatial_split_dim(SpatialSplit *p_ss) {
	thread_local SpatialBin spatial_bins[kSpatialBinNum];
	thread_local AABB right_aabbs[kSpatialBinNum];

	std::fill(spatial_bins, spatial_bins + kSpatialBinNum, SpatialBin{AABB(), 0, 0}); // initialize bins
	auto &node = access_node(m_node_idx);
	const float bin_width = node.aabb.GetExtent()[DIM] / kSpatialBinNum, inv_bin_width = 1.0f / bin_width;
	const float bound_base = node.aabb.min[DIM];

	// Put references into bins
	for (uint32_t ref_idx : m_references) {
		const auto &ref = access_reference(ref_idx);
		uint32_t bin = glm::clamp(uint32_t((ref.aabb.min[DIM] - bound_base) * inv_bin_width), 0u, kSpatialBinNum - 1);
		uint32_t last_bin =
		    glm::clamp(uint32_t((ref.aabb.max[DIM] - bound_base) * inv_bin_width), 0u, kSpatialBinNum - 1);

		spatial_bins[bin].in++;
		Reference cur_ref = ref;
		for (; bin < last_bin; ++bin) {
			auto [left_ref, right_ref] =
			    m_p_builder->split_reference(cur_ref, DIM, float(bin + 1) * bin_width + bound_base);
			spatial_bins[bin].aabb.Expand(left_ref.aabb);
			cur_ref = right_ref;
		}
		spatial_bins[last_bin].aabb.Expand(cur_ref.aabb);
		spatial_bins[last_bin].out++;
	}

	// Compute the AABBs from right
	right_aabbs[kSpatialBinNum - 1] = spatial_bins[kSpatialBinNum - 1].aabb;
	for (int32_t i = kSpatialBinNum - 2; i >= 1; --i)
		right_aabbs[i] = AABB(spatial_bins[i].aabb, right_aabbs[i + 1]);

	// Find optimal spatial split
	AABB left_aabb = spatial_bins[0].aabb;
	uint32_t left_num = 0, right_num = m_references.size();
	for (uint32_t i = 1; i < kSpatialBinNum; ++i) {
		left_num += spatial_bins[i - 1].in;
		right_num -= spatial_bins[i - 1].out;

		float sah = float(left_num) * left_aabb.GetHalfArea() + float(right_num) * right_aabbs[i].GetHalfArea();
		if (sah < p_ss->sah && left_num > 0 && right_num > 0) {
			p_ss->sah = sah;
			p_ss->dim = DIM;
			p_ss->pos = bound_base + float(i) * bin_width;
		}

		left_aabb.Expand(spatial_bins[i].aabb);
	}
}
void ParallelSBVHBuilder::Task::_find_spatial_split_parallel(ParallelSBVHBuilder::Task::SpatialSplit *p_ss) {
	auto &node = access_node(m_node_idx);

	const glm::vec3 &bin_bases = node.aabb.min;
	const glm::vec3 bin_widths = node.aabb.GetExtent() / (float)kSpatialBinNum, inv_bin_widths = 1.0f / bin_widths;
	std::atomic_uint32_t counter{0};

	auto compute_spatial_bins_func = [this, &bin_bases, &bin_widths, &inv_bin_widths, &counter]() {
		std::array<std::array<ParallelSBVHBuilder::Task::SpatialBin, ParallelSBVHBuilder::kSpatialBinNum>, 3> ret{};

		for (uint32_t cur_block = counter++; cur_block * kParallelForBlockSize < m_references.size();
		     cur_block = counter++) {
			uint32_t cur_first = cur_block * kParallelForBlockSize,
			         cur_last = std::min((cur_block + 1) * kParallelForBlockSize, (uint32_t)m_references.size());

			for (uint32_t cur = cur_first; cur < cur_last; ++cur) {
				const auto &ref = access_reference(m_references[cur]);
				glm::u32vec3 bins =
				    glm::clamp(glm::u32vec3((ref.aabb.min - bin_bases) * inv_bin_widths), 0u, kSpatialBinNum - 1);
				glm::u32vec3 last_bins =
				    glm::clamp(glm::u32vec3((ref.aabb.max - bin_bases) * inv_bin_widths), 0u, kSpatialBinNum - 1);

				for (int dim = 0; dim < 3; ++dim) {
					uint32_t bin = bins[dim], last_bin = last_bins[dim];
					auto &spatial_bins = ret[dim];

					++spatial_bins[bin].in;
					Reference cur_ref = ref;
					for (; bin < last_bin; ++bin) {
						auto [left_ref, right_ref] = m_p_builder->split_reference(
						    cur_ref, dim, float(bin + 1) * bin_widths[dim] + bin_bases[dim]);
						spatial_bins[bin].aabb.Expand(left_ref.aabb);
						cur_ref = right_ref;
					}
					spatial_bins[last_bin].aabb.Expand(cur_ref.aabb);
					++spatial_bins[last_bin].out;
				}
			}
		}
		return ret;
	};

	// Async compute bins
	std::vector<std::future<std::array<std::array<SpatialBin, kSpatialBinNum>, 3>>> futures(m_thread_count - 1);
	for (uint32_t i = 1; i < m_thread_count; ++i)
		futures[i - 1] = get_thread_unit(i).Push(compute_spatial_bins_func);
	auto bins = compute_spatial_bins_func();

	// Merge Bins
	for (auto &f : futures) {
		auto r = f.get();
		for (int dim = 0; dim < 3; ++dim) {
			for (uint32_t x = 0; x < kSpatialBinNum; ++x) {
				auto &cl = bins[dim][x];
				const auto &cr = r[dim][x];
				cl.aabb.Expand(cr.aabb);
				cl.in += cr.in;
				cl.out += cr.out;
			}
		}
	}

	thread_local AABB right_aabbs[kSpatialBinNum];
	for (int dim = 0; dim < 3; ++dim) {
		const auto &dim_bins = bins[dim];

		right_aabbs[kSpatialBinNum - 1] = dim_bins[kSpatialBinNum - 1].aabb;
		for (int32_t i = kSpatialBinNum - 2; i >= 1; --i)
			right_aabbs[i] = AABB(dim_bins[i].aabb, right_aabbs[i + 1]);

		// Find optimal spatial split
		AABB left_aabb = dim_bins[0].aabb;
		uint32_t left_num = 0, right_num = m_references.size();
		for (uint32_t i = 1; i < kSpatialBinNum; ++i) {
			left_num += dim_bins[i - 1].in;
			right_num -= dim_bins[i - 1].out;

			float sah = float(left_num) * left_aabb.GetHalfArea() + float(right_num) * right_aabbs[i].GetHalfArea();
			if (sah < p_ss->sah && left_num > 0 && right_num > 0) {
				p_ss->sah = sah;
				p_ss->dim = dim;
				p_ss->pos = bin_bases[dim] + float(i) * bin_widths[dim];
			}

			left_aabb.Expand(dim_bins[i].aabb);
		}
	}
}
ParallelSBVHBuilder::Task::SpatialSplit ParallelSBVHBuilder::Task::find_spatial_split() {
	SpatialSplit ss{};
	if (m_thread_count > 1) {
		_find_spatial_split_parallel(&ss);
	} else {
		_find_spatial_split_dim<0>(&ss);
		_find_spatial_split_dim<1>(&ss);
		_find_spatial_split_dim<2>(&ss);
	}
	return ss;
}
std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::perform_spatial_split(const SpatialSplit &ss) {
	auto &node = access_node(m_node_idx);
	if (!node.left)
		node.left = new_node();
	if (!node.right)
		node.right = new_node();

	auto &left_node = access_node(node.left), &right_node = access_node(node.right);
	left_node.aabb = right_node.aabb = AABB();

	// separate the bound into 3 parts
	//[left_begin, left_end) - totally left part
	//[left_end, right_begin) - the part to split
	//[right_begin, right_end) - totally right part
	constexpr uint32_t left_begin = 0;
	uint32_t left_end = 0, right_begin = m_references.size(), right_end = m_references.size();
	for (uint32_t i = left_begin; i < right_begin; ++i) {
		// put to left
		const auto &ref = access_reference(m_references[i]);
		if (ref.aabb.max[(int)ss.dim] <= ss.pos) {
			left_node.aabb.Expand(ref.aabb);
			std::swap(m_references[i], m_references[left_end++]);
		} else if (ref.aabb.min[(int)ss.dim] >= ss.pos) {
			right_node.aabb.Expand(ref.aabb);
			std::swap(m_references[i--], m_references[--right_begin]);
		}
	}

	if ((left_begin == left_end || right_begin == right_end) && left_end == right_begin) {
		return {};
	}

	AABB lub; // Unsplit to left:     new left-hand bounds.
	AABB rub; // Unsplit to right:    new right-hand bounds.
	AABB lsb; // Split:               new left-hand bounds.
	AABB rsb; // Split:               new right-hand bounds.
	while (left_end < right_begin) {
		auto &cur_ref = access_reference(m_references[left_end]);
		auto [left_ref, right_ref] = m_p_builder->split_reference(cur_ref, ss.dim, ss.pos);

		lub = lsb = left_node.aabb;
		rub = rsb = right_node.aabb;

		lub.Expand(cur_ref.aabb);
		rub.Expand(cur_ref.aabb);
		lsb.Expand(left_ref.aabb);
		rsb.Expand(right_ref.aabb);

		auto lac = float(left_end - left_begin);
		auto rac = float(right_end - right_begin);
		auto lbc = float(1 + left_end - left_begin);
		auto rbc = float(1 + right_end - right_begin);

		float unsplit_left_sah = lub.GetHalfArea() * lbc + right_node.aabb.GetHalfArea() * rac;
		float unsplit_right_sah = left_node.aabb.GetHalfArea() * lac + rub.GetHalfArea() * rbc;
		float split_sah = lsb.GetHalfArea() * lbc + rsb.GetHalfArea() * rbc;

		if (unsplit_left_sah < unsplit_right_sah && unsplit_left_sah < split_sah &&
		    right_begin < right_end) { // unsplit to left
			left_node.aabb = lub;
			++left_end;
		} else if (unsplit_right_sah < split_sah && left_begin < left_end) { // unsplit to right
			right_node.aabb = rub;
			std::swap(m_references[left_end], m_references[--right_begin]);
		} else { // duplicate
			m_references.emplace_back();
			left_node.aabb = lsb;
			right_node.aabb = rsb;

			cur_ref = left_ref;
			uint32_t right_ref_idx = new_reference();
			access_reference(right_ref_idx) = right_ref;
			// m_references[left_end++] = p_left_ref;
			m_references[right_end++] = right_ref_idx;
		}
	}

	assert(left_begin < left_end && right_begin < right_end);

	std::vector<uint32_t> &left_refs = m_references;
	std::vector<uint32_t> right_refs{m_references.begin() + right_begin, m_references.begin() + right_end};
	left_refs.resize(left_end - left_begin);

	auto [left_thread_count, right_thread_count] = get_child_thread_counts(left_refs.size(), right_refs.size());
	return {Task{m_p_builder, node.left, std::move(left_refs), m_depth + 1, m_thread, left_thread_count},
	        Task{m_p_builder, node.right, std::move(right_refs), m_depth + 1, m_thread + left_thread_count,
	             right_thread_count}};
}

/*
 Object split
 */
template <uint32_t DIM> void ParallelSBVHBuilder::Task::_find_swept_object_split_dim(ObjectSplit *p_sos) {
	// Sort first
	m_p_builder->sort_references<DIM>(m_references.begin(), m_references.end());

	// Preprocess right_aabbs
	thread_local std::vector<AABB> right_aabbs{};

	if (right_aabbs.size() < m_references.size())
		right_aabbs.resize(m_references.size());
	right_aabbs[m_references.size() - 1] = access_reference(m_references.back()).aabb;
	for (int32_t i = (int32_t)m_references.size() - 2; i >= 1; --i)
		right_aabbs[i] = AABB(access_reference(m_references[i]).aabb, right_aabbs[i + 1]);

	// Find optimal object split
	AABB left_aabb = access_reference(m_references.front()).aabb;
	for (uint32_t i = 1; i < m_references.size(); ++i) {
		float sah = float(i) * left_aabb.GetHalfArea() + float(m_references.size() - i) * right_aabbs[i].GetHalfArea();
		if (sah < p_sos->sah) {
			p_sos->use_bins = false;
			p_sos->sah = sah;
			p_sos->dim = DIM;
			p_sos->left_aabb = left_aabb;
			p_sos->right_aabb = right_aabbs[i];

			p_sos->swept.pos = (access_reference(m_references[i - 1]).aabb.GetDimCenter<DIM>() +
			                    access_reference(m_references[i]).aabb.GetDimCenter<DIM>()) *
			                   0.5f;
			p_sos->swept.left_num = i;
		}

		left_aabb.Expand(access_reference(m_references[i]).aabb);
	}
}
template <uint32_t DIM> void ParallelSBVHBuilder::Task::_find_binned_object_split_dim(ObjectSplit *p_bos) {
	thread_local ObjectBin object_bins[kObjectBinNum];
	thread_local AABB right_aabbs[kObjectBinNum];

	float bound_min = FLT_MAX, bound_max = -FLT_MAX;
	for (uint32_t ref_idx : m_references) {
		const auto &ref = access_reference(ref_idx);
		float c = ref.aabb.GetDimCenter<DIM>();
		bound_max = std::max(bound_max, c);
		bound_min = std::min(bound_min, c);
	}

	std::fill(object_bins, object_bins + kObjectBinNum, ObjectBin{AABB(), 0}); // initialize bins
	const float bin_width = (bound_max - bound_min) / kObjectBinNum, inv_bin_width = 1.0f / bin_width;
	const float bin_base = bound_min; // m_node->aabb.min[DIM];

	// Put references into bins according to centers
	for (uint32_t ref_idx : m_references) {
		const auto &ref = access_reference(ref_idx);
		uint32_t bin =
		    glm::clamp(uint32_t((ref.aabb.GetDimCenter<DIM>() - bin_base) * inv_bin_width), 0u, kObjectBinNum - 1);
		object_bins[bin].aabb.Expand(ref.aabb);
		++object_bins[bin].cnt;
	}

	// Compute the AABBs from right
	right_aabbs[kObjectBinNum - 1] = object_bins[kObjectBinNum - 1].aabb;
	for (int32_t i = kObjectBinNum - 2; i >= 1; --i)
		right_aabbs[i] = AABB(object_bins[i].aabb, right_aabbs[i + 1]);

	// Find optimal object split
	AABB left_aabb = object_bins[0].aabb;
	uint32_t left_num = 0;
	for (uint32_t i = 1; i < kObjectBinNum; ++i) {
		left_num += object_bins[i - 1].cnt;
		uint32_t right_num = m_references.size() - left_num;

		float sah = float(left_num) * left_aabb.GetHalfArea() + float(right_num) * right_aabbs[i].GetHalfArea();
		if (sah < p_bos->sah && left_num > 0 && right_num > 0) {
			p_bos->use_bins = true;
			p_bos->sah = sah;
			p_bos->left_aabb = left_aabb;
			p_bos->right_aabb = right_aabbs[i];
			p_bos->dim = DIM;

			p_bos->binned.bin_pos = i;
			p_bos->binned.bin_base = bin_base;
			p_bos->binned.inv_bin_width = inv_bin_width;
		}

		left_aabb.Expand(object_bins[i].aabb);
	}
}
void ParallelSBVHBuilder::Task::_find_binned_object_split_parallel(ObjectSplit *p_bos) {
	AABB center_bound;
	{ // Parallel compute center bound
		std::atomic_uint32_t counter{0};
		auto compute_center_bound_func = [this, &counter]() {
			AABB ret{};
			for (uint32_t cur_block = counter++; cur_block * kParallelForBlockSize < m_references.size();
			     cur_block = counter++) {
				uint32_t cur_first = cur_block * kParallelForBlockSize,
				         cur_last = std::min((cur_block + 1) * kParallelForBlockSize, (uint32_t)m_references.size());
				for (uint32_t cur = cur_first; cur < cur_last; ++cur) {
					const auto &ref = access_reference(m_references[cur]);
					ret.Expand(ref.aabb.GetCenter());
				}
			}
			return ret;
		};
		std::vector<std::future<AABB>> futures(m_thread_count - 1);
		for (uint32_t i = 1; i < m_thread_count; ++i)
			futures[i - 1] = get_thread_unit(i).Push(compute_center_bound_func);
		center_bound = compute_center_bound_func();
		for (auto &f : futures)
			center_bound.Expand(f.get());
	}

	const glm::vec3 &bin_bases = center_bound.min;
	const glm::vec3 bin_widths = center_bound.GetExtent() / (float)kObjectBinNum, inv_bin_widths = 1.0f / bin_widths;

	std::atomic_uint32_t counter{0};
	auto compute_object_bins_func = [this, &bin_bases, &bin_widths, &inv_bin_widths, &counter]() {
		std::array<std::array<ObjectBin, kObjectBinNum>, 3> ret{};

		for (uint32_t cur_block = counter++; cur_block * kParallelForBlockSize < m_references.size();
		     cur_block = counter++) {
			uint32_t cur_first = cur_block * kParallelForBlockSize,
			         cur_last = std::min((cur_block + 1) * kParallelForBlockSize, (uint32_t)m_references.size());

			for (uint32_t cur = cur_first; cur < cur_last; ++cur) {
				const auto &ref = access_reference(m_references[cur]);
				glm::u32vec3 bins = glm::clamp(glm::u32vec3((ref.aabb.GetCenter() - bin_bases) * inv_bin_widths), 0u,
				                               kObjectBinNum - 1);
				for (int dim = 0; dim < 3; ++dim) {
					auto &object_bins = ret[dim];
					uint32_t bin = bins[dim];

					++object_bins[bin].cnt;
					object_bins[bin].aabb.Expand(ref.aabb);
				}
			}
		}
		return ret;
	};

	// Async compute bins
	std::vector<std::future<std::array<std::array<ObjectBin, kObjectBinNum>, 3>>> futures(m_thread_count - 1);
	for (uint32_t i = 1; i < m_thread_count; ++i)
		futures[i - 1] = get_thread_unit(i).Push(compute_object_bins_func);
	auto bins = compute_object_bins_func();

	// Merge Bins
	for (auto &f : futures) {
		auto r = f.get();
		for (int dim = 0; dim < 3; ++dim) {
			for (uint32_t x = 0; x < kObjectBinNum; ++x) {
				auto &cl = bins[dim][x];
				const auto &cr = r[dim][x];
				cl.aabb.Expand(cr.aabb);
				cl.cnt += cr.cnt;
			}
		}
	}

	thread_local AABB right_aabbs[kObjectBinNum];
	for (int dim = 0; dim < 3; ++dim) {
		const auto &dim_bins = bins[dim];

		right_aabbs[kObjectBinNum - 1] = dim_bins[kObjectBinNum - 1].aabb;
		for (int32_t i = kObjectBinNum - 2; i >= 1; --i)
			right_aabbs[i] = AABB(dim_bins[i].aabb, right_aabbs[i + 1]);

		// Find optimal object split
		AABB left_aabb = dim_bins[0].aabb;
		uint32_t left_num = 0;
		for (uint32_t i = 1; i < kObjectBinNum; ++i) {
			left_num += dim_bins[i - 1].cnt;
			uint32_t right_num = m_references.size() - left_num;

			float sah = float(left_num) * left_aabb.GetHalfArea() + float(right_num) * right_aabbs[i].GetHalfArea();
			if (sah < p_bos->sah && left_num > 0 && right_num > 0) {
				p_bos->use_bins = true;
				p_bos->sah = sah;
				p_bos->left_aabb = left_aabb;
				p_bos->right_aabb = right_aabbs[i];
				p_bos->dim = dim;

				p_bos->binned.bin_pos = i;
				p_bos->binned.inv_bin_width = inv_bin_widths[dim];
				p_bos->binned.bin_base = bin_bases[dim];
			}

			left_aabb.Expand(dim_bins[i].aabb);
		}
	}
}

ParallelSBVHBuilder::Task::ObjectSplit ParallelSBVHBuilder::Task::find_object_split() {
	ObjectSplit os{};
	if (m_references.size() >= kObjectBinNum) {
		if (m_thread_count > 1)
			_find_binned_object_split_parallel(&os);
		else {
			_find_binned_object_split_dim<0>(&os);
			_find_binned_object_split_dim<1>(&os);
			_find_binned_object_split_dim<2>(&os);
		}
	}
	if (os.sah == FLT_MAX) {
		_find_swept_object_split_dim<0>(&os);
		_find_swept_object_split_dim<1>(&os);
		_find_swept_object_split_dim<2>(&os);
	}
	return os;
}

std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::perform_object_split(const ObjectSplit &os) {
	return os.use_bins ? _perform_binned_object_split(os) : _perform_swept_object_split(os);
}

std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::_perform_binned_object_split(const ObjectSplit &bos) {
	auto &node = access_node(m_node_idx);
	if (!node.left)
		node.left = new_node();
	if (!node.right)
		node.right = new_node();
	auto &left_node = access_node(node.left), &right_node = access_node(node.right);
	left_node.aabb = right_node.aabb = AABB();

	uint32_t left_num = 0;
	for (uint32_t i = 0; i < m_references.size(); ++i) {
		const auto &ref = access_reference(m_references[i]);
		uint32_t bin =
		    glm::clamp(uint32_t((ref.aabb.GetDimCenter((int)bos.dim) - bos.binned.bin_base) * bos.binned.inv_bin_width),
		               0u, kObjectBinNum - 1);
		if (bin < bos.binned.bin_pos) {
			left_node.aabb.Expand(ref.aabb);
			std::swap(m_references[i], m_references[left_num++]);
		} else
			right_node.aabb.Expand(ref.aabb);
	}

	if (left_num == 0 || left_num == m_references.size()) {
		spdlog::error("Why?");
	}

	std::vector<uint32_t> &left_refs = m_references;
	std::vector<uint32_t> right_refs{m_references.begin() + left_num, m_references.end()};
	left_refs.resize(left_num);

	auto [left_thread_count, right_thread_count] = get_child_thread_counts(left_refs.size(), right_refs.size());
	return {Task{m_p_builder, node.left, std::move(left_refs), m_depth + 1, m_thread, left_thread_count},
	        Task{m_p_builder, node.right, std::move(right_refs), m_depth + 1, m_thread + left_thread_count,
	             right_thread_count}};
}
std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::_perform_swept_object_split(const ParallelSBVHBuilder::Task::ObjectSplit &sos) {
	auto &node = access_node(m_node_idx);
	if (!node.left)
		node.left = new_node();
	if (!node.right)
		node.right = new_node();
	auto &left_node = access_node(node.left), &right_node = access_node(node.right);
	left_node.aabb = right_node.aabb = AABB();

	uint32_t left_num = 0;
	for (uint32_t i = 0; i < m_references.size(); ++i) {
		const auto &ref = access_reference(m_references[i]);
		float c = ref.aabb.GetDimCenter((int)sos.dim);
		if (c <= sos.swept.pos && left_num < sos.swept.left_num) {
			left_node.aabb.Expand(ref.aabb);
			std::swap(m_references[i], m_references[left_num++]);
		} else
			right_node.aabb.Expand(ref.aabb);
	}

	std::vector<uint32_t> &left_refs = m_references;
	std::vector<uint32_t> right_refs{m_references.begin() + left_num, m_references.end()};
	left_refs.resize(left_num);

	auto [left_thread_count, right_thread_count] = get_child_thread_counts(left_refs.size(), right_refs.size());
	return {Task{m_p_builder, node.left, std::move(left_refs), m_depth + 1, m_thread, left_thread_count},
	        Task{m_p_builder, node.right, std::move(right_refs), m_depth + 1, m_thread + left_thread_count,
	             right_thread_count}};
}
