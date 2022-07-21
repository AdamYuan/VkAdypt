#include "ParallelSBVHBuilder.hpp"

#include <future>
#include <pdqsort.h>
#include <spdlog/spdlog.h>

void ParallelSBVHBuilder::Run() {
	spdlog::info("Begin");
	push_root_task();
	{
		std::vector<std::future<void>> workers(std::thread::hardware_concurrency());
		for (auto &worker : workers) {
			worker = std::async([this]() -> void {
				moodycamel::ConsumerToken consumer_token{m_task_queue};
				moodycamel::ProducerToken producer_token{m_task_queue};
				auto node_allocator = LocalAllocator{m_node_allocator};

				Task task;
				while (m_task_count.load()) {
					if (m_task_queue.try_dequeue(consumer_token, task)) {
						auto new_tasks = task.Run(&node_allocator);
						if (std::get<0>(new_tasks).Empty()) {
							--m_task_count;
						} else {
							++m_task_count;
							m_node_count.fetch_add(2);
							m_task_queue.enqueue(producer_token, std::get<0>(new_tasks));
							m_task_queue.enqueue(producer_token, std::get<1>(new_tasks));
						}
					}
				}
			});
		}
	}
	spdlog::info("End");
}

std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::Run(LocalAllocator<Node, kAllocatorChunkSize> *p_node_allocator) {
	if (m_references.size() == 1) {
		perform_leaf();
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
		auto ret = perform_spatial_split(p_node_allocator, spatial_split);
		if (!std::get<0>(ret).Empty()) {
			return ret;
		}
	}
	return perform_object_split(p_node_allocator, object_split);
}

void ParallelSBVHBuilder::push_root_task() {
	m_root.aabb = m_scene.GetAABB();
	std::vector<Reference> references;
	{
		references.reserve(m_scene.GetTriangles().size());
		for (uint32_t i = 0; i < m_scene.GetTriangles().size(); ++i) {
			Reference ref{};
			ref.tri_idx = i;
			ref.aabb = m_scene.GetTriangles()[i].GetAABB();
			references.push_back(ref);
		}
	}
	m_task_count.store(1);
	m_task_queue.enqueue(Task{this, &m_root, std::move(references), 0});
	m_node_count.store(1);
}

template <uint32_t DIM> bool ParallelSBVHBuilder::reference_cmp(const Reference &l, const Reference &r) {
	float lc = l.aabb.GetCenter()[DIM], rc = r.aabb.GetCenter()[DIM];
	return std::tie(lc, l.tri_idx) < std::tie(rc, r.tri_idx);
}
template <uint32_t DIM> void ParallelSBVHBuilder::sort_references(std::vector<Reference> *references) {
	pdqsort(references->begin(), references->end(), reference_cmp<DIM>);
}
void ParallelSBVHBuilder::sort_references(std::vector<Reference> *references, uint32_t dim) {
	pdqsort(references->begin(), references->end(),
	        dim != 0 ? (dim == 1 ? reference_cmp<1> : reference_cmp<2>) : reference_cmp<0>);
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

uint32_t ParallelSBVHBuilder::fetch_result(ParallelSBVHBuilder::Node *cur, std::vector<BinaryBVH::Node> *p_nodes,
                                           uint32_t *p_leaf_count) {
	uint32_t cur_bin_idx = p_nodes->size();
	p_nodes->emplace_back();
	auto &cur_bin_node = p_nodes->back();
	cur_bin_node.aabb = cur->aabb;
	if (!cur->left || !cur->right) { // Leaf
		cur_bin_node.left_idx = UINT32_MAX;
		cur_bin_node.tri_idx = cur->tri_idx;
		++(*p_leaf_count);
	} else {
		fetch_result(cur->right, p_nodes, p_leaf_count);
		cur_bin_node.left_idx = fetch_result(cur->left, p_nodes, p_leaf_count);
	}
	return cur_bin_idx;
}

void ParallelSBVHBuilder::FetchResult(std::vector<BinaryBVH::Node> *p_nodes, uint32_t *p_leaf_count) {
	p_nodes->clear();
	p_nodes->reserve(m_node_count.load());
	*p_leaf_count = 0;
	fetch_result(&m_root, p_nodes, p_leaf_count);
	spdlog::info("Leaf count: {}", *p_leaf_count);
}

/*
 Spatial split
 */
struct SpatialBin {
	AABB aabb;
	uint32_t in{}, out{};
};
template <uint32_t DIM>
void ParallelSBVHBuilder::Task::_find_spatial_split_dim(ParallelSBVHBuilder::Task::SpatialSplit *p_ss) {
	thread_local SpatialBin spatial_bins[kSpatialBinNum];
	thread_local AABB right_aabbs[kSpatialBinNum];

	std::fill(spatial_bins, spatial_bins + kSpatialBinNum, SpatialBin{AABB(), 0, 0}); // initialize bins
	const float bin_width = m_node->aabb.GetExtent()[DIM] / kSpatialBinNum, inv_bin_width = 1.0f / bin_width;
	const float bound_base = m_node->aabb.min[DIM];

	// Put references into bins
	for (const auto &ref : m_references) {
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

		float sah = float(left_num) * left_aabb.GetArea() + float(right_num) * right_aabbs[i].GetArea();
		if (sah < p_ss->sah && left_num > 0 && right_num > 0) {
			p_ss->sah = sah;
			p_ss->dim = DIM;
			p_ss->pos = bound_base + float(i) * bin_width;
		}

		left_aabb.Expand(spatial_bins[i].aabb);
	}
}
ParallelSBVHBuilder::Task::SpatialSplit ParallelSBVHBuilder::Task::find_spatial_split() {
	SpatialSplit ss{};
	_find_spatial_split_dim<0>(&ss);
	_find_spatial_split_dim<1>(&ss);
	_find_spatial_split_dim<2>(&ss);
	return ss;
}
std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::perform_spatial_split(LocalAllocator<Node, kAllocatorChunkSize> *p_node_allocator,
                                                 const ParallelSBVHBuilder::Task::SpatialSplit &ss) {
	if (!m_node->left)
		m_node->left = p_node_allocator->Alloc();
	if (!m_node->right)
		m_node->right = p_node_allocator->Alloc();

	auto &left = m_node->left, &right = m_node->right;
	left->aabb = right->aabb = AABB();

	// separate the bound into 3 parts
	//[left_begin, left_end) - totally left part
	//[left_end, right_begin) - the part to split
	//[right_begin, right_end) - totally right part
	constexpr uint32_t left_begin = 0;
	uint32_t left_end = 0, right_begin = m_references.size(), right_end = m_references.size();
	for (uint32_t i = left_begin; i < right_begin; ++i) {
		// put to left
		const auto &ref = m_references[i];
		if (ref.aabb.max[(int)ss.dim] <= ss.pos) {
			left->aabb.Expand(ref.aabb);
			std::swap(m_references[i], m_references[left_end++]);
		} else if (ref.aabb.min[(int)ss.dim] >= ss.pos) {
			right->aabb.Expand(ref.aabb);
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
		auto [left_ref, right_ref] = m_p_builder->split_reference(m_references[left_end], ss.dim, ss.pos);

		lub = lsb = left->aabb;
		rub = rsb = right->aabb;

		lub.Expand(m_references[left_end].aabb);
		rub.Expand(m_references[left_end].aabb);
		lsb.Expand(left_ref.aabb);
		rsb.Expand(right_ref.aabb);

		auto lac = float(left_end - left_begin);
		auto rac = float(right_end - right_begin);
		auto lbc = float(1 + left_end - left_begin);
		auto rbc = float(1 + right_end - right_begin);

		float unsplit_left_sah = lub.GetArea() * lbc + right->aabb.GetArea() * rac;
		float unsplit_right_sah = left->aabb.GetArea() * lac + rub.GetArea() * rbc;
		float split_sah = lsb.GetArea() * lbc + rsb.GetArea() * rbc;

		if (unsplit_left_sah < unsplit_right_sah && unsplit_left_sah < split_sah &&
		    right_begin < right_end) { // unsplit to left
			left->aabb = lub;
			++left_end;
		} else if (unsplit_right_sah < split_sah && left_begin < left_end) { // unsplit to right
			right->aabb = rub;
			std::swap(m_references[left_end], m_references[--right_begin]);
		} else { // duplicate
			m_references.emplace_back();
			left->aabb = lsb;
			right->aabb = rsb;
			m_references[left_end++] = left_ref;
			m_references[right_end++] = right_ref;
		}
	}

	assert(left_begin < left_end && right_begin < right_end);

	std::vector<Reference> &left_refs = m_references;
	std::vector<Reference> right_refs{m_references.begin() + right_begin, m_references.begin() + right_end};
	left_refs.resize(left_end - left_begin);
	left_refs.shrink_to_fit();

	return {Task{m_p_builder, left, std::move(left_refs), m_depth + 1},
	        Task{m_p_builder, right, std::move(right_refs), m_depth + 1}};
}

/*
 Object split
 */
template <uint32_t DIM>
void ParallelSBVHBuilder::Task::_find_object_split_dim(ParallelSBVHBuilder::Task::ObjectSplit *p_os) {
	// Sort first
	sort_references<DIM>(&m_references);

	// Preprocess right_aabbs
	thread_local std::vector<AABB> right_aabbs{};
	if (right_aabbs.size() < m_references.size())
		right_aabbs.resize(m_references.size());
	right_aabbs[m_references.size() - 1] = m_references.back().aabb;
	for (int32_t i = (int32_t)m_references.size() - 2; i >= 1; --i)
		right_aabbs[i] = AABB(m_references[i].aabb, right_aabbs[i + 1]);

	// Find optimal object split
	AABB left_aabb = m_references.front().aabb;
	for (uint32_t i = 1; i < m_references.size(); ++i) {
		float sah = float(i) * left_aabb.GetArea() + float(m_references.size() - i) * right_aabbs[i].GetArea();
		if (sah < p_os->sah) {
			p_os->dim = DIM;
			p_os->left_num = i;
			p_os->left_aabb = left_aabb;
			p_os->right_aabb = right_aabbs[i];
			p_os->sah = sah;
		}

		left_aabb.Expand(m_references[i].aabb);
	}
}
ParallelSBVHBuilder::Task::ObjectSplit ParallelSBVHBuilder::Task::find_object_split() {
	ObjectSplit os{};
	_find_object_split_dim<0>(&os);
	_find_object_split_dim<1>(&os);
	_find_object_split_dim<2>(&os);
	return os;
}
std::tuple<ParallelSBVHBuilder::Task, ParallelSBVHBuilder::Task>
ParallelSBVHBuilder::Task::perform_object_split(LocalAllocator<Node, kAllocatorChunkSize> *p_node_allocator,
                                                const ParallelSBVHBuilder::Task::ObjectSplit &os) {
	sort_references(&m_references, os.dim);

	if (!m_node->left)
		m_node->left = p_node_allocator->Alloc();
	if (!m_node->right)
		m_node->right = p_node_allocator->Alloc();

	auto &left = m_node->left, &right = m_node->right;
	left->aabb = os.left_aabb;
	right->aabb = os.right_aabb;

	std::vector<Reference> &left_refs = m_references;
	std::vector<Reference> right_refs{m_references.begin() + os.left_num, m_references.end()};
	left_refs.resize(os.left_num);
	left_refs.shrink_to_fit();

	return {Task{m_p_builder, left, std::move(left_refs), m_depth + 1},
	        Task{m_p_builder, right, std::move(right_refs), m_depth + 1}};
}
