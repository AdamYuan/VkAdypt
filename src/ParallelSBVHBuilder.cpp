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
	m_thread_reference_block_allocators.reserve(kThreadCount);
	for (uint32_t i = 0; i < kThreadCount; ++i) {
		m_consumer_tokens.emplace_back(m_task_queue);
		m_producer_tokens.emplace_back(m_task_queue);
		m_thread_node_allocators.emplace_back(m_bvh.m_node_pool);
		m_thread_reference_allocators.emplace_back(m_reference_pool);
		if (i == 0)
			m_thread_reference_block_allocators.emplace_back(
			    m_reference_block_pool, GetReferenceBlockSize(m_scene.GetTriangles().size()) << 1u);
		else
			m_thread_reference_block_allocators.emplace_back(m_reference_block_pool);
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
	auto [reference_block, tmp_reference_block, reference_block_size] =
	    alloc_reference_block(m_thread_reference_block_allocators.data(), m_scene.GetTriangles().size());
	{
		for (uint32_t i = 0; i < m_scene.GetTriangles().size(); ++i) {
			uint32_t ref_idx = m_thread_reference_allocators[0].Alloc();
			auto &ref = m_reference_pool[ref_idx];
			ref.tri_idx = i;
			ref.aabb = m_scene.GetTriangles()[i].GetAABB();
			reference_block[i] = ref_idx;
		}
	}
	return Task{this,
	            root_idx,
	            Task::kAlignLeft,
	            (uint32_t)m_scene.GetTriangles().size(),
	            reference_block_size,
	            reference_block,
	            tmp_reference_block,
	            0,
	            0,
	            kThreadCount};
}

std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task> ParallelSBVHBuilder::Task::Run() {
	if (m_reference_count == 1) {
		make_leaf();
		return {};
	}

	ObjectSplit object_split = find_object_split();
	SpatialSplit spatial_split{};
	if (m_depth <= m_p_builder->m_config.m_max_spatial_depth) {
		AABB overlap = object_split.left_aabb;
		overlap.IntersectAABB(object_split.right_aabb);
		if (overlap.GetArea() >= m_p_builder->m_min_overlap_area)
			spatial_split = find_spatial_split();
	}
	if (spatial_split.sah < object_split.sah) {
		auto ret = perform_spatial_split(spatial_split);
		if (!PairEmpty(ret))
			return ret;
	}
	{
		auto ret = perform_object_split(object_split);
		if (!PairEmpty(ret))
			return ret;
	}
	return perform_default_split();
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
				task.AssignToThread(m_thread);

				if (m_reference_count <= kLocalRunThreshold) {
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

uint32_t ParallelSBVHBuilder::Task::split_thread(uint32_t left_ref_count, uint32_t right_ref_count) const {
	if (m_thread_count == 0)
		return 0u;
	auto lt = (float)left_ref_count, tt = float(left_ref_count + right_ref_count);
	return std::clamp(uint32_t(glm::round(lt / tt * float(m_thread_count))), 0u, m_thread_count);
}

uint32_t ParallelSBVHBuilder::Task::split_reference_block(uint32_t left_ref_count, uint32_t right_ref_count,
                                                          uint32_t ref_block_size) {
	auto lt = (float)left_ref_count, tt = float(left_ref_count + right_ref_count);
	return std::clamp(uint32_t(glm::round(lt / tt * float(ref_block_size))), left_ref_count,
	                  ref_block_size - right_ref_count);
}
std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::split_task_with_block(uint32_t left_ref_count, uint32_t right_ref_count,
                                                 uint32_t ref_block_size, uint32_t *ref_block,
                                                 uint32_t *tmp_ref_block) const {
	uint32_t left_thread_count = split_thread(left_ref_count, right_ref_count);
	uint32_t left_ref_block_size = split_reference_block(left_ref_count, right_ref_count, ref_block_size);
	auto &node = access_node(m_node_index);
	return {Task{m_p_builder, node.left, kAlignLeft, left_ref_count, left_ref_block_size, ref_block, tmp_ref_block,
	             m_depth + 1, m_thread, left_thread_count},
	        Task{m_p_builder, node.right, kAlignRight, right_ref_count, ref_block_size - left_ref_block_size,
	             ref_block + left_ref_block_size, tmp_ref_block + left_ref_block_size, m_depth + 1,
	             m_thread + left_thread_count, m_thread_count - left_thread_count}};
}
std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::split_task(uint32_t left_ref_count, uint32_t right_ref_count, bool swap_to_tmp) const {
	if (swap_to_tmp)
		return split_task_with_block(left_ref_count, right_ref_count, m_reference_block_size, m_tmp_reference_block,
		                             m_reference_block);
	else
		return split_task_with_block(left_ref_count, right_ref_count, m_reference_block_size, m_reference_block,
		                             m_tmp_reference_block);
}
/*
 Spatial split
 */
template <uint32_t DIM> void ParallelSBVHBuilder::Task::_find_spatial_split_dim(SpatialSplit *p_ss) {
	thread_local SpatialBin spatial_bins[kSpatialBinNum];
	thread_local AABB right_aabbs[kSpatialBinNum];

	std::fill(spatial_bins, spatial_bins + kSpatialBinNum, SpatialBin{AABB(), 0, 0}); // initialize bins
	auto &node = access_node(m_node_index);
	const float bin_width = node.aabb.GetExtent()[DIM] / kSpatialBinNum, inv_bin_width = 1.0f / bin_width;
	const float bound_base = node.aabb.min[DIM];

	// Put references into bins
	auto ref_begin = get_reference_begin();
	for (uint32_t i = 0; i < m_reference_count; ++i) {
		const auto &ref = access_reference(ref_begin[i]);
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
	uint32_t left_num = 0, right_num = m_reference_count;
	for (uint32_t i = 1; i < kSpatialBinNum; ++i) {
		left_num += spatial_bins[i - 1].in;
		right_num -= spatial_bins[i - 1].out;

		float sah = float(left_num) * left_aabb.GetArea() + float(right_num) * right_aabbs[i].GetArea();
		if (sah < p_ss->sah && left_num > 0 && right_num > 0) {
			p_ss->sah = sah;
			p_ss->dim = DIM;
			p_ss->pos = bound_base + float(i) * bin_width;
			p_ss->ref_cnt = left_num + right_num;
		}

		left_aabb.Expand(spatial_bins[i].aabb);
	}
}
void ParallelSBVHBuilder::Task::_find_spatial_split_parallel(ParallelSBVHBuilder::Task::SpatialSplit *p_ss) {
	auto &node = access_node(m_node_index);

	const glm::vec3 &bin_bases = node.aabb.min;
	const glm::vec3 bin_widths = node.aabb.GetExtent() / (float)kSpatialBinNum, inv_bin_widths = 1.0f / bin_widths;
	std::atomic_uint32_t counter{0};

	auto ref_begin = get_reference_begin();
	auto compute_spatial_bins_func = [this, ref_begin, &bin_bases, &bin_widths, &inv_bin_widths, &counter]() {
		std::array<std::array<ParallelSBVHBuilder::Task::SpatialBin, ParallelSBVHBuilder::kSpatialBinNum>, 3> ret{};

		for (uint32_t cur_block = counter++; cur_block * kParallelForBlockSize < m_reference_count;
		     cur_block = counter++) {
			uint32_t cur_first = cur_block * kParallelForBlockSize,
			         cur_last = std::min((cur_block + 1) * kParallelForBlockSize, m_reference_count);

			for (uint32_t cur = cur_first; cur < cur_last; ++cur) {
				const auto &ref = access_reference(ref_begin[cur]);
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
		uint32_t left_num = 0, right_num = m_reference_count;
		for (uint32_t i = 1; i < kSpatialBinNum; ++i) {
			left_num += dim_bins[i - 1].in;
			right_num -= dim_bins[i - 1].out;

			float sah = float(left_num) * left_aabb.GetArea() + float(right_num) * right_aabbs[i].GetArea();
			if (sah < p_ss->sah && left_num > 0 && right_num > 0) {
				p_ss->sah = sah;
				p_ss->dim = dim;
				p_ss->pos = bin_bases[dim] + float(i) * bin_widths[dim];
				p_ss->ref_cnt = left_num + right_num;
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
	return _perform_spatial_split(ss);
	// return m_thread_count > 1 ? _perform_spatial_split_parallel(ss) : _perform_spatial_split(ss);
}
std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::_perform_spatial_split(const SpatialSplit &ss) {
	auto [left_node, right_node] = maintain_child_nodes();
	left_node.aabb = right_node.aabb = AABB();

	auto ref_begin = get_reference_begin();

	uint32_t *ref_block = m_reference_block, *tmp_ref_block = m_tmp_reference_block,
	         ref_block_size = m_reference_block_size;
	if (ss.ref_cnt > m_reference_block_size) {
		std::tie(ref_block, tmp_ref_block, ref_block_size) = new_reference_block(ss.ref_cnt);
		if (ref_block == nullptr)
			return {};
	}
	uint32_t *tmp_ref_block_begin = tmp_ref_block, *tmp_ref_block_end = tmp_ref_block + ref_block_size;

	uint32_t left_num = 0, right_num = 0;
	uint32_t split_num = 0; // The number of references to splitted
	for (uint32_t i = 0; i < m_reference_count; ++i) {
		uint32_t ref_idx = ref_begin[i];
		const auto &ref = access_reference(ref_idx);
		if (ref.aabb.max[(int)ss.dim] <= ss.pos) {
			left_node.aabb.Expand(ref.aabb);
			*(tmp_ref_block_begin + (left_num++)) = ref_idx;
		} else if (ref.aabb.min[(int)ss.dim] >= ss.pos) {
			right_node.aabb.Expand(ref.aabb);
			*(tmp_ref_block_end - (++right_num)) = ref_idx;
		} else
			std::swap(ref_begin[i], ref_begin[split_num++]);
	}

	if ((left_num == 0 || right_num == 0) && split_num == 0)
		return {};

	AABB lub; // Unsplit to left:     new left-hand bounds.
	AABB rub; // Unsplit to right:    new right-hand bounds.
	AABB lsb; // Split:               new left-hand bounds.
	AABB rsb; // Split:               new right-hand bounds.
	for (uint32_t i = 0; i < split_num; ++i) {
		uint32_t ref_idx = ref_begin[i];
		const auto &ref = access_reference(ref_idx);
		auto [left_ref, right_ref] = m_p_builder->split_reference(ref, ss.dim, ss.pos);

		AABB lb = AABB{left_node.aabb, ref.aabb};
		AABB rb = AABB{right_node.aabb, ref.aabb};
		float left_sah = lb.GetArea() * float(1 + left_num) + right_node.aabb.GetArea() * float(right_num);
		float right_sah = left_node.aabb.GetArea() * float(left_num) + rb.GetArea() * float(1 + right_num);

		lub = lsb = left_node.aabb;
		rub = rsb = right_node.aabb;

		lub.Expand(ref.aabb);
		rub.Expand(ref.aabb);
		lsb.Expand(left_ref.aabb);
		rsb.Expand(right_ref.aabb);

		float unsplit_left_sah = lub.GetArea() * float(1 + left_num) + right_node.aabb.GetArea() * float(right_num);
		float unsplit_right_sah = left_node.aabb.GetArea() * float(left_num) + rub.GetArea() * float(1 + right_num);
		float split_sah = lsb.GetArea() * float(1 + left_num) + rsb.GetArea() * float(1 + right_num);

		if (unsplit_left_sah < unsplit_right_sah && unsplit_left_sah < split_sah && right_num > 0) { // unsplit to left
			left_node.aabb = lub;
			*(tmp_ref_block_begin + (left_num++)) = ref_idx;
		} else if (unsplit_right_sah < split_sah && left_num > 0) { // unsplit to right
			right_node.aabb = rub;
			*(tmp_ref_block_end - (++right_num)) = ref_idx;
		} else { // duplicate
			left_node.aabb = lsb;
			right_node.aabb = rsb;

			access_reference(ref_idx) = left_ref;
			uint32_t right_ref_idx = new_reference();
			access_reference(right_ref_idx) = right_ref;

			*(tmp_ref_block_begin + (left_num++)) = ref_idx;
			*(tmp_ref_block_end - (++right_num)) = right_ref_idx;
		}
	}
	/* for (uint32_t i = 0; i < split_num; ++i) {
	    uint32_t ref_idx = ref_begin[i];
	    const auto &ref = access_reference(ref_idx);
	    auto [left_ref, right_ref] = m_p_builder->split_reference(ref, ss.dim, ss.pos);

	    left_node.aabb.Expand(left_ref.aabb);
	    right_node.aabb.Expand(right_ref.aabb);

	    access_reference(ref_idx) = left_ref;
	    uint32_t right_ref_idx = new_reference();
	    access_reference(right_ref_idx) = right_ref;

	    push_ref_block_left(tmp_ref_block_begin, left_num++, ref_idx);
	    push_ref_block_right(tmp_ref_block_end, right_num++, right_ref_idx);
	} */

	if (left_num == 0 || right_num == 0)
		return {};

	return split_task_with_block(left_num, right_num, ref_block_size, tmp_ref_block, ref_block);
}
/* std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::_perform_spatial_split_parallel(const SpatialSplit &ss) {
    auto &node = access_node(m_node_idx);
    if (!node.left)
        node.left = new_node();
    if (!node.right)
        node.right = new_node();

    auto &left_node = access_node(node.left), &right_node = access_node(node.right);
    left_node.aabb = right_node.aabb = AABB();

    std::atomic_uint32_t left_counter = 0, right_counter = 0;
    std::vector<uint32_t> &tmp_references = get_tmp_references(m_references.size());

    AABB left_aabb, right_aabb;
    uint32_t left_num, right_num, origin_num = m_references.size();
    std::vector<std::vector<uint32_t>> extra_right_references;
    extra_right_references.reserve(m_thread_count);
    extra_right_references.emplace_back();
    {
        std::atomic_uint32_t counter = 0;
        auto left_right_spatial_split_func = [this, &ss, &counter, &left_counter, &right_counter,
                                              &tmp_references](uint32_t thread_idx) {
            std::tuple<AABB, AABB, std::vector<uint32_t>> ret{};
            AABB &left_aabb = std::get<0>(ret);
            AABB &right_aabb = std::get<1>(ret);
            std::vector<uint32_t> &right_extra_refs = std::get<2>(ret);
            left_aabb = {};
            right_aabb = {};

            for (uint32_t cur_block = counter++; cur_block * kParallelForBlockSize < m_references.size();
                 cur_block = counter++) {
                uint32_t cur_first = cur_block * kParallelForBlockSize,
                         cur_last = std::min((cur_block + 1) * kParallelForBlockSize, (uint32_t)m_references.size());

                for (uint32_t cur = cur_first; cur < cur_last; ++cur) {
                    const auto &ref = access_reference(m_references[cur]);

                    if (ref.aabb.max[(int)ss.dim] <= ss.pos) {
                        left_aabb.Expand(ref.aabb);
                        tmp_references[left_counter++] = m_references[cur];
                    } else if (ref.aabb.min[(int)ss.dim] >= ss.pos) {
                        right_aabb.Expand(ref.aabb);
                        tmp_references[m_references.size() - (++right_counter)] = m_references[cur];
                    } else {
                        auto [left_ref, right_ref] = m_p_builder->split_reference(ref, ss.dim, ss.pos);
                        left_aabb.Expand(left_ref.aabb);
                        right_aabb.Expand(right_ref.aabb);

                        uint32_t left_ref_idx = m_references[cur];
                        access_reference(left_ref_idx) = left_ref;
                        uint32_t right_ref_idx = new_reference(thread_idx);
                        access_reference(right_ref_idx) = right_ref;
                        tmp_references[left_counter++] = left_ref_idx;
                        right_extra_refs.push_back(right_ref_idx);
                    }
                }
            }
            return ret;
        };
        std::vector<std::future<std::tuple<AABB, AABB, std::vector<uint32_t>>>> futures(m_thread_count - 1);
        for (uint32_t i = 1; i < m_thread_count; ++i)
            futures[i - 1] = get_thread_unit(i).Push(left_right_spatial_split_func, i);
        std::tie(left_aabb, right_aabb, extra_right_references[0]) = left_right_spatial_split_func(0);
        right_num = extra_right_references[0].size();
        // Merge AABBs
        for (auto &f : futures) {
            auto [f_left_aabb, f_right_aabb, f_extra_refs] = f.get();
            right_num += f_extra_refs.size();
            extra_right_references.push_back(std::move(f_extra_refs));
            left_aabb.Expand(f_left_aabb);
            right_aabb.Expand(f_right_aabb);
        }
        left_num = left_counter;
        right_num += right_counter;
    }
    left_node.aabb = left_aabb;
    right_node.aabb = right_aabb;

    if (left_num == 0 || right_num == 0)
        return {};

    m_references.clear();
    m_references.shrink_to_fit();

    std::vector<uint32_t> left_refs = {tmp_references.begin(), tmp_references.begin() + left_num};
    std::vector<uint32_t> right_refs;
    right_refs.reserve(right_num);
    std::copy(tmp_references.begin() + left_num, tmp_references.begin() + origin_num, std::back_inserter(right_refs));
    for (auto &extra_right_refs : extra_right_references)
        std::copy(extra_right_refs.begin(), extra_right_refs.end(), std::back_inserter(right_refs));

    auto [left_thread_count, right_thread_count] = get_child_thread_counts(left_refs.size(), right_refs.size());
    return {Task{m_p_builder, node.left, std::move(left_refs), m_depth + 1, m_thread, left_thread_count},
            Task{m_p_builder, node.right, std::move(right_refs), m_depth + 1, m_thread + left_thread_count,
                 right_thread_count}};
} */

/*
 Object split
 */
template <uint32_t DIM> void ParallelSBVHBuilder::Task::_find_object_split_sweep_dim(ObjectSplit *p_os) {
	// Sort first
	auto ref_begin = get_reference_begin();
	m_p_builder->sort_references<DIM>(ref_begin, ref_begin + m_reference_count);

	// Preprocess right_aabbs
	thread_local std::vector<AABB> right_aabbs{};

	if (right_aabbs.size() < m_reference_count)
		right_aabbs.resize(m_reference_count);
	right_aabbs[m_reference_count - 1] = access_reference(ref_begin[m_reference_count - 1]).aabb;
	for (int32_t i = (int32_t)m_reference_count - 2; i >= 1; --i)
		right_aabbs[i] = AABB(access_reference(ref_begin[i]).aabb, right_aabbs[i + 1]);

	// Find optimal object split
	AABB left_aabb = access_reference(ref_begin[0]).aabb;
	for (uint32_t i = 1; i < m_reference_count; ++i) {
		float sah = float(i) * left_aabb.GetArea() + float(m_reference_count - i) * right_aabbs[i].GetArea();
		if (sah < p_os->sah) {
			p_os->dim = DIM;
			p_os->pos = (access_reference(ref_begin[i - 1]).aabb.GetDimCenter<DIM>() +
			             access_reference(ref_begin[i]).aabb.GetDimCenter<DIM>()) *
			            0.5f;
			p_os->left_aabb = left_aabb;
			p_os->right_aabb = right_aabbs[i];
			p_os->sah = sah;
			p_os->bin_width = 0.0f;
		}

		left_aabb.Expand(access_reference(ref_begin[i]).aabb);
	}
}
template <uint32_t DIM> void ParallelSBVHBuilder::Task::_find_object_split_binned_dim(ObjectSplit *p_os) {
	thread_local ObjectBin object_bins[kObjectBinNum];
	thread_local AABB right_aabbs[kObjectBinNum];

	auto ref_begin = get_reference_begin();

	float bound_min = FLT_MAX, bound_max = -FLT_MAX;
	for (uint32_t i = 0; i < m_reference_count; ++i) {
		const auto &ref = access_reference(ref_begin[i]);
		float c = ref.aabb.GetDimCenter<DIM>();
		bound_max = std::max(bound_max, c);
		bound_min = std::min(bound_min, c);
	}

	std::fill(object_bins, object_bins + kObjectBinNum, ObjectBin{AABB(), 0}); // initialize bins
	const float bin_width = (bound_max - bound_min) / kObjectBinNum, inv_bin_width = 1.0f / bin_width;
	const float bound_base = bound_min; // m_node->aabb.min[DIM];

	// Put references into bins according to centers
	for (uint32_t i = 0; i < m_reference_count; ++i) {
		const auto &ref = access_reference(ref_begin[i]);
		uint32_t bin =
		    glm::clamp(uint32_t((ref.aabb.GetDimCenter<DIM>() - bound_base) * inv_bin_width), 0u, kObjectBinNum - 1);
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
		uint32_t right_num = m_reference_count - left_num;

		float sah = float(left_num) * left_aabb.GetArea() + float(right_num) * right_aabbs[i].GetArea();
		if (sah < p_os->sah && left_num > 0 && right_num > 0) {
			p_os->left_aabb = left_aabb;
			p_os->right_aabb = right_aabbs[i];
			p_os->sah = sah;
			p_os->dim = DIM;
			p_os->pos = bound_base + float(i) * bin_width;
			p_os->bin_width = bin_width;
		}

		left_aabb.Expand(object_bins[i].aabb);
	}
}
void ParallelSBVHBuilder::Task::_find_object_split_binned_parallel(ObjectSplit *p_os) {
	auto ref_begin = get_reference_begin();

	AABB center_bound;
	{ // Parallel compute center bound
		std::atomic_uint32_t counter{0};
		auto compute_center_bound_func = [this, ref_begin, &counter]() {
			AABB ret{};
			for (uint32_t cur_block = counter++; cur_block * kParallelForBlockSize < m_reference_count;
			     cur_block = counter++) {
				uint32_t cur_first = cur_block * kParallelForBlockSize,
				         cur_last = std::min((cur_block + 1) * kParallelForBlockSize, m_reference_count);
				for (uint32_t cur = cur_first; cur < cur_last; ++cur) {
					const auto &ref = access_reference(ref_begin[cur]);
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
	auto compute_object_bins_func = [this, ref_begin, &bin_bases, &bin_widths, &inv_bin_widths, &counter]() {
		std::array<std::array<ObjectBin, kObjectBinNum>, 3> ret{};

		for (uint32_t cur_block = counter++; cur_block * kParallelForBlockSize < m_reference_count;
		     cur_block = counter++) {
			uint32_t cur_first = cur_block * kParallelForBlockSize,
			         cur_last = std::min((cur_block + 1) * kParallelForBlockSize, m_reference_count);

			for (uint32_t cur = cur_first; cur < cur_last; ++cur) {
				const auto &ref = access_reference(ref_begin[cur]);
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
			uint32_t right_num = m_reference_count - left_num;

			float sah = float(left_num) * left_aabb.GetArea() + float(right_num) * right_aabbs[i].GetArea();
			if (sah < p_os->sah && left_num > 0 && right_num > 0) {
				p_os->left_aabb = left_aabb;
				p_os->right_aabb = right_aabbs[i];
				p_os->sah = sah;
				p_os->dim = dim;
				p_os->pos = bin_bases[dim] + float(i) * bin_widths[dim];
				p_os->bin_width = bin_widths[dim];
			}

			left_aabb.Expand(dim_bins[i].aabb);
		}
	}
}
ParallelSBVHBuilder::Task::ObjectSplit ParallelSBVHBuilder::Task::find_object_split() {
	ObjectSplit os{};
	if (m_reference_count >= kObjectBinNum) {
		if (m_thread_count > 1)
			_find_object_split_binned_parallel(&os);
		else {
			_find_object_split_binned_dim<0>(&os);
			_find_object_split_binned_dim<1>(&os);
			_find_object_split_binned_dim<2>(&os);
		}
	}
	// Fallback to sweep SAH if failed
	if (os.sah == FLT_MAX) {
		_find_object_split_sweep_dim<0>(&os);
		_find_object_split_sweep_dim<1>(&os);
		_find_object_split_sweep_dim<2>(&os);
	}
	return os;
}
std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::perform_object_split(const ObjectSplit &os) {
	return _perform_object_split(os);
	// return m_thread_count > 1 ? _perform_object_split_parallel(os) : _perform_object_split(os);
}
std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::_perform_object_split(const ObjectSplit &os) {
	auto [left_node, right_node] = maintain_child_nodes();
	left_node.aabb = right_node.aabb = AABB();

	uint32_t *ref_begin = get_reference_begin();
	uint32_t *tmp_ref_block_begin = m_tmp_reference_block,
	         *tmp_ref_block_end = m_tmp_reference_block + m_reference_block_size;

	const float delta = 0.5f * os.bin_width;

	uint32_t left_num = 0, right_num = 0;
	uint32_t tbd_num = 0; // The number of references to be determined
	for (uint32_t i = 0; i < m_reference_count; ++i) {
		uint32_t ref_idx = ref_begin[i];
		const auto &ref = access_reference(ref_idx);
		float c = ref.aabb.GetDimCenter((int)os.dim);
		if (c < os.pos - delta) {
			left_node.aabb.Expand(ref.aabb);
			*(tmp_ref_block_begin + (left_num++)) = ref_idx;
		} else if (c > os.pos + delta) {
			right_node.aabb.Expand(ref.aabb);
			*(tmp_ref_block_end - (++right_num)) = ref_idx;
		} else
			std::swap(ref_begin[i], ref_begin[tbd_num++]);
	}

	if ((left_num == 0 || right_num == 0) && tbd_num == 0)
		return {};

	std::shuffle(ref_begin, ref_begin + tbd_num, std::minstd_rand{});
	for (uint32_t i = 0; i < tbd_num; ++i) {
		uint32_t ref_idx = ref_begin[i];
		const auto &ref = access_reference(ref_idx);

		AABB lb = AABB{left_node.aabb, ref.aabb};
		AABB rb = AABB{right_node.aabb, ref.aabb};
		float left_sah = lb.GetArea() * float(1 + left_num) + right_node.aabb.GetArea() * float(right_num);
		float right_sah = left_node.aabb.GetArea() * float(left_num) + rb.GetArea() * float(1 + right_num);

		if (left_sah < right_sah || left_num == 0) { // unsplit to left
			left_node.aabb = lb;
			*(tmp_ref_block_begin + (left_num++)) = ref_idx;
		} else {
			right_node.aabb = rb;
			*(tmp_ref_block_end - (++right_num)) = ref_idx;
		}
	}

	if (left_num == 0 || right_num == 0)
		return {};

	return split_task(left_num, right_num, true);
}
/* std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::_perform_object_split_parallel(const ObjectSplit &os) {
    return _perform_object_split(os);
    auto &node = access_node(m_node_idx);
    if (!node.left)
        node.left = new_node();
    if (!node.right)
        node.right = new_node();

    auto &left_node = access_node(node.left), &right_node = access_node(node.right);
    left_node.aabb = right_node.aabb = AABB();
}*/

std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task> ParallelSBVHBuilder::Task::perform_default_split() {
	spdlog::warn("Default split, {}", m_reference_count);

	auto [left_node, right_node] = maintain_child_nodes();

	uint32_t left_num = m_reference_count >> 1u, right_num = m_reference_count - left_num;
	auto ref_begin = get_reference_begin();
	left_node.aabb = access_reference(ref_begin[0]).aabb;
	for (uint32_t i = 1; i < left_num; ++i)
		left_node.aabb.Expand(access_reference(ref_begin[i]).aabb);

	right_node.aabb = access_reference(ref_begin[left_num]).aabb;
	for (uint32_t i = left_num + 1; i < m_reference_count; ++i)
		right_node.aabb.Expand(access_reference(ref_begin[i]).aabb);

	if (m_reference_alignment == kAlignLeft)
		std::copy(ref_begin + left_num, ref_begin + m_reference_count,
		          m_reference_block + m_reference_block_size - right_num);
	else
		std::copy(ref_begin, ref_begin + left_num, m_reference_block);

	return split_task(left_num, right_num, false);
}
