#include "BVHConfig.hpp"
#include "SBVH.hpp"
#include <algorithm>
#include <chrono>
#include <spdlog/spdlog.h>

class SBVHBuilder {
private:
	static constexpr uint32_t kSpatialBinNum = 32;

	struct Reference {
		AABB m_aabb;
		uint32_t m_tri_index;
	};
	struct NodeSpec {
		AABB m_aabb;
		uint32_t m_ref_num;
	};
	struct ObjectSplit {
		AABB m_left_aabb, m_right_aabb;
		uint32_t m_dim, m_left_num;
		float m_sah;
	};
	struct SpatialSplit {
		uint32_t m_dim;
		float m_pos, m_sah;
	};
	struct SpatialBin {
		AABB m_aabb;
		uint32_t m_in, m_out;
	};

	SBVH *m_bvh;
	const Scene &m_scene;
	const BVHConfig &m_config;

	SpatialBin m_spatial_bins[kSpatialBinNum];

	std::vector<AABB> m_right_aabbs; // store right aabb result
	std::vector<Reference> m_refstack;

	float m_min_overlap_area;

	// for std::sort
	template <uint32_t DIM> inline static bool reference_cmp(const Reference &l, const Reference &r);
	template <uint32_t DIM> inline void sort_spec(const NodeSpec &t_spec);
	inline void sort_spec(const NodeSpec &t_spec, uint32_t dim);
	inline uint32_t get_ref_index(const NodeSpec &t_spec) { return (uint32_t)m_refstack.size() - t_spec.m_ref_num; }
	inline uint32_t build_leaf(const NodeSpec &t_spec);

	template <uint32_t DIM> inline void __find_object_split_dim(const NodeSpec &t_spec, ObjectSplit *t_os);
	inline void find_object_split(const NodeSpec &t_spec, ObjectSplit *t_os);
	inline void perform_object_split(const NodeSpec &t_spec, const ObjectSplit &t_os, NodeSpec *t_left,
	                                 NodeSpec *t_right);
	inline void split_reference(const Reference &t_ref, uint32_t t_dim, float t_pos, Reference *t_left,
	                            Reference *t_right);
	template <uint32_t DIM> inline void __find_spatial_split_dim(const NodeSpec &t_spec, SpatialSplit *t_ss);
	inline void find_spatial_split(const NodeSpec &t_spec, SpatialSplit *t_ss);
	inline void perform_spatial_split(const NodeSpec &t_spec, const SpatialSplit &t_ss, NodeSpec *t_left,
	                                  NodeSpec *t_right);
	uint32_t build_node(const NodeSpec &t_spec, uint32_t t_depth);
	inline uint32_t push_node() {
		m_bvh->m_nodes.emplace_back();
		return m_bvh->m_nodes.size() - 1;
	}

public:
	SBVHBuilder(SBVH *bvh, const Scene &scene) : m_scene(scene), m_bvh(bvh), m_config(bvh->m_config) {
		m_bvh->m_leaves_cnt = 0;
		m_bvh->m_nodes.clear();
	}
	void Run();
};

std::shared_ptr<SBVH> SBVH::Build(const BVHConfig &config, const std::shared_ptr<Scene> &scene) {
	std::shared_ptr<SBVH> ret = std::make_shared<SBVH>();
	ret->m_config = config;
	ret->m_scene_ptr = scene;
	SBVHBuilder builder{ret.get(), *scene};
	builder.Run();
	return ret;
}

template <uint32_t DIM>
bool SBVHBuilder::reference_cmp(const SBVHBuilder::Reference &l, const SBVHBuilder::Reference &r) {
	float lc = l.m_aabb.GetCenter()[DIM], rc = r.m_aabb.GetCenter()[DIM];
	return lc < rc || (lc == rc && l.m_tri_index < r.m_tri_index);
}

template <uint32_t DIM> void SBVHBuilder::sort_spec(const SBVHBuilder::NodeSpec &t_spec) {
	std::sort(m_refstack.data() + m_refstack.size() - t_spec.m_ref_num, m_refstack.data() + m_refstack.size(),
	          reference_cmp<DIM>);
}

void SBVHBuilder::sort_spec(const SBVHBuilder::NodeSpec &t_spec, uint32_t dim) {
	std::sort(m_refstack.data() + m_refstack.size() - t_spec.m_ref_num, m_refstack.data() + m_refstack.size(),
	          dim != 0 ? (dim == 1 ? reference_cmp<1> : reference_cmp<2>) : reference_cmp<0>);
}

uint32_t SBVHBuilder::build_leaf(const SBVHBuilder::NodeSpec &t_spec) {
	uint32_t node = push_node();
	m_bvh->m_nodes[node].m_aabb = t_spec.m_aabb;
	m_bvh->m_nodes[node].m_left_idx = UINT32_MAX; // mark to -1 for leaf
	m_bvh->m_nodes[node].m_tri_idx = m_refstack.back().m_tri_index;
	m_refstack.pop_back();
	return node;
}

template <uint32_t DIM>
void SBVHBuilder::__find_object_split_dim(const SBVHBuilder::NodeSpec &t_spec, SBVHBuilder::ObjectSplit *t_os) {
	sort_spec<DIM>(t_spec);
	Reference *refs = m_refstack.data() + get_ref_index(t_spec);

	// get the aabb from right
	m_right_aabbs.resize((size_t)t_spec.m_ref_num);
	m_right_aabbs[t_spec.m_ref_num - 1] = refs[t_spec.m_ref_num - 1].m_aabb;
	for (uint32_t i = t_spec.m_ref_num - 2; i >= 1; --i)
		m_right_aabbs[i] = AABB(refs[i].m_aabb, m_right_aabbs[i + 1]);

	AABB left_aabb = refs->m_aabb;
	for (uint32_t i = 1; i <= t_spec.m_ref_num - 1; ++i) {
		float sah = i * left_aabb.GetArea() + (t_spec.m_ref_num - i) * m_right_aabbs[i].GetArea();
		if (sah < t_os->m_sah) {
			t_os->m_dim = DIM;
			t_os->m_left_num = i;
			t_os->m_left_aabb = left_aabb;
			t_os->m_right_aabb = m_right_aabbs[i];
			t_os->m_sah = sah;
		}

		left_aabb.Expand(refs[i].m_aabb);
	}
}

void SBVHBuilder::find_object_split(const SBVHBuilder::NodeSpec &t_spec, SBVHBuilder::ObjectSplit *t_os) {
	t_os->m_sah = FLT_MAX;

	__find_object_split_dim<0>(t_spec, t_os);
	__find_object_split_dim<1>(t_spec, t_os);
	__find_object_split_dim<2>(t_spec, t_os);
}

void SBVHBuilder::split_reference(const SBVHBuilder::Reference &t_ref, uint32_t t_dim, float t_pos,
                                  SBVHBuilder::Reference *t_left, SBVHBuilder::Reference *t_right)
// if the part is invalid, set m_tri_index to -1
{
	t_left->m_aabb = t_right->m_aabb = AABB();
	t_left->m_tri_index = t_right->m_tri_index = t_ref.m_tri_index;

	const Triangle &tri = m_scene.GetTriangles()[t_ref.m_tri_index];
	for (uint32_t i = 0; i < 3; ++i) {
		const glm::vec3 &v0 = tri.m_positions[i], &v1 = tri.m_positions[(i + 1) % 3];
		float p0 = v0[t_dim], p1 = v1[t_dim];
		if (p0 <= t_pos)
			t_left->m_aabb.Expand(v0);
		if (p0 >= t_pos)
			t_right->m_aabb.Expand(v0);

		if ((p0 < t_pos && t_pos < p1) || (p1 < t_pos && t_pos < p0)) // process edge
		{
			glm::vec3 x = glm::mix(v0, v1, glm::clamp((t_pos - p0) / (p1 - p0), 0.0f, 1.0f));
			t_left->m_aabb.Expand(x);
			t_right->m_aabb.Expand(x);
		}
	}

	t_left->m_aabb.m_max[t_dim] = t_pos;
	t_left->m_aabb.IntersectAABB(t_ref.m_aabb);

	t_right->m_aabb.m_min[t_dim] = t_pos;
	t_right->m_aabb.IntersectAABB(t_ref.m_aabb);
}

template <uint32_t DIM>
void SBVHBuilder::__find_spatial_split_dim(const SBVHBuilder::NodeSpec &t_spec, SBVHBuilder::SpatialSplit *t_ss) {
	std::fill(m_spatial_bins, m_spatial_bins + kSpatialBinNum, SpatialBin{AABB(), 0, 0}); // initialize bins

	float bin_width = t_spec.m_aabb.GetExtent()[DIM] / kSpatialBinNum, inv_bin_width = 1.0f / bin_width;
	float bound_base = t_spec.m_aabb.m_min[DIM];
	Reference *refs = m_refstack.data() + get_ref_index(t_spec);
	Reference cur_ref, left_ref, right_ref;

	// put references into bins
	for (uint32_t i = 0; i < t_spec.m_ref_num; ++i) {
		uint32_t bin =
		    glm::clamp(uint32_t((refs[i].m_aabb.m_min[DIM] - bound_base) * inv_bin_width), 0u, kSpatialBinNum - 1);
		uint32_t last_bin =
		    glm::clamp(uint32_t((refs[i].m_aabb.m_max[DIM] - bound_base) * inv_bin_width), 0u, kSpatialBinNum - 1);
		m_spatial_bins[bin].m_in++;
		cur_ref = refs[i];
		for (; bin < last_bin; ++bin) {
			split_reference(cur_ref, DIM, (bin + 1) * bin_width + bound_base, &left_ref, &right_ref);
			m_spatial_bins[bin].m_aabb.Expand(left_ref.m_aabb);
			cur_ref = right_ref;
		}
		m_spatial_bins[last_bin].m_aabb.Expand(cur_ref.m_aabb);
		m_spatial_bins[last_bin].m_out++;
	}

	// get the aabb from right
	m_right_aabbs.resize(kSpatialBinNum);
	m_right_aabbs[kSpatialBinNum - 1] = m_spatial_bins[kSpatialBinNum - 1].m_aabb;
	for (uint32_t i = kSpatialBinNum - 2; i >= 1; --i)
		m_right_aabbs[i] = AABB(m_spatial_bins[i].m_aabb, m_right_aabbs[i + 1]);

	// compare sah
	AABB left_aabb = m_spatial_bins[0].m_aabb;
	uint32_t left_num = 0, right_num = t_spec.m_ref_num;
	for (uint32_t i = 1; i < kSpatialBinNum; ++i) {
		left_num += m_spatial_bins[i - 1].m_in;
		right_num -= m_spatial_bins[i - 1].m_out;

		float sah = (left_num)*left_aabb.GetArea() + (right_num)*m_right_aabbs[i].GetArea();
		if (sah < t_ss->m_sah) {
			t_ss->m_sah = sah;
			t_ss->m_dim = DIM;
			t_ss->m_pos = bound_base + i * bin_width;
		}

		left_aabb.Expand(m_spatial_bins[i].m_aabb);
	}
}

void SBVHBuilder::find_spatial_split(const SBVHBuilder::NodeSpec &t_spec, SBVHBuilder::SpatialSplit *t_ss) {
	t_ss->m_sah = FLT_MAX;
	__find_spatial_split_dim<0>(t_spec, t_ss);
	__find_spatial_split_dim<1>(t_spec, t_ss);
	__find_spatial_split_dim<2>(t_spec, t_ss);
}

void SBVHBuilder::perform_spatial_split(const SBVHBuilder::NodeSpec &t_spec, const SBVHBuilder::SpatialSplit &t_ss,
                                        SBVHBuilder::NodeSpec *t_left, SBVHBuilder::NodeSpec *t_right) {
	t_left->m_aabb = t_right->m_aabb = AABB();

	uint32_t refs = get_ref_index(t_spec);
	// separate the bound into 3 parts
	//[left_begin, left_end) - totally left part
	//[left_end, right_begin) - the part to split
	//[right_begin, right_end) - totally right part
	uint32_t left_begin = 0, left_end = 0, right_begin = t_spec.m_ref_num, right_end = t_spec.m_ref_num;
	for (uint32_t i = left_begin; i < right_begin; ++i) {
		// put to left
		if (m_refstack[refs + i].m_aabb.m_max[t_ss.m_dim] <= t_ss.m_pos) {
			t_left->m_aabb.Expand(m_refstack[refs + i].m_aabb);
			std::swap(m_refstack[refs + i], m_refstack[refs + (left_end++)]);
		} else if (m_refstack[refs + i].m_aabb.m_min[t_ss.m_dim] >= t_ss.m_pos) {
			t_right->m_aabb.Expand(m_refstack[refs + i].m_aabb);
			std::swap(m_refstack[refs + (i--)], m_refstack[refs + (--right_begin)]);
		}
	}

	Reference left_ref, right_ref;

	AABB lub; // Unsplit to left:     new left-hand bounds.
	AABB rub; // Unsplit to right:    new right-hand bounds.
	AABB ldb; // Duplicate:           new left-hand bounds.
	AABB rdb; // Duplicate:           new right-hand bounds.

	while (left_end < right_begin) {
		split_reference(m_refstack[refs + left_end], t_ss.m_dim, t_ss.m_pos, &left_ref, &right_ref);

		lub = ldb = t_left->m_aabb;
		rub = rdb = t_right->m_aabb;

		lub.Expand(m_refstack[refs + left_end].m_aabb);
		rub.Expand(m_refstack[refs + left_end].m_aabb);
		ldb.Expand(left_ref.m_aabb);
		rdb.Expand(right_ref.m_aabb);

		float lac = left_end - left_begin;
		float rac = right_end - right_begin;
		float lbc = 1 + left_end - left_begin;
		float rbc = 1 + right_end - right_begin;

		float unsplit_left_sah = lub.GetArea() * lbc + t_right->m_aabb.GetArea() * rac;
		float unsplit_right_sah = t_left->m_aabb.GetArea() * lac + rub.GetArea() * rbc;
		float duplicate_sah = ldb.GetArea() * lbc + rdb.GetArea() * rbc;

		if (unsplit_left_sah < unsplit_right_sah && unsplit_left_sah < duplicate_sah) { // unsplit left
			t_left->m_aabb = lub;
			left_end++;
		} else if (unsplit_right_sah < duplicate_sah) { // unsplit right
			t_right->m_aabb = rub;
			std::swap(m_refstack[refs + left_end], m_refstack[refs + (--right_begin)]);
		} else { // duplicate
			m_refstack.emplace_back();
			t_left->m_aabb = ldb;
			t_right->m_aabb = rdb;
			m_refstack[refs + (left_end++)] = left_ref;
			m_refstack[refs + (right_end++)] = right_ref;
		}
	}

	t_left->m_ref_num = left_end - left_begin;
	t_right->m_ref_num = right_end - right_begin;
}

void SBVHBuilder::perform_object_split(const SBVHBuilder::NodeSpec &t_spec, const SBVHBuilder::ObjectSplit &t_os,
                                       SBVHBuilder::NodeSpec *t_left, SBVHBuilder::NodeSpec *t_right) {
	sort_spec(t_spec, t_os.m_dim);

	t_left->m_ref_num = t_os.m_left_num;
	t_left->m_aabb = t_os.m_left_aabb;

	t_right->m_ref_num = t_spec.m_ref_num - t_os.m_left_num;
	t_right->m_aabb = t_os.m_right_aabb;
}

uint32_t SBVHBuilder::build_node(const NodeSpec &t_spec, uint32_t t_depth) {
	if (t_spec.m_ref_num == 1)
		return build_leaf(t_spec);

	float area = t_spec.m_aabb.GetArea();

	ObjectSplit object_split;
	find_object_split(t_spec, &object_split);

	SpatialSplit spatial_split;
	spatial_split.m_sah = FLT_MAX;
	if (t_depth <= m_config.m_max_spatial_depth) {
		AABB overlap = object_split.m_left_aabb;
		overlap.IntersectAABB(object_split.m_right_aabb);
		if (overlap.GetArea() >= m_min_overlap_area)
			find_spatial_split(t_spec, &spatial_split);
	}

	uint32_t node = push_node(); // alloc new node
	m_bvh->m_nodes[node].m_aabb = t_spec.m_aabb;

	NodeSpec left, right;
	left.m_ref_num = right.m_ref_num = 0;
	if (spatial_split.m_sah < object_split.m_sah)
		perform_spatial_split(t_spec, spatial_split, &left, &right);

	if (left.m_ref_num == 0 || right.m_ref_num == 0)
		perform_object_split(t_spec, object_split, &left, &right);

	build_node(right, t_depth + 1);
	// use a temp variable to get the return value()
	uint32_t lidx = build_node(left, t_depth + 1);
	m_bvh->m_nodes[node].m_left_idx = lidx;

	return node;
}

void SBVHBuilder::Run() {
	m_right_aabbs.reserve(m_scene.GetTriangles().size());

	// init reference stack
	m_refstack.reserve(m_scene.GetTriangles().size() * 2);
	m_refstack.resize(m_scene.GetTriangles().size());
	for (uint32_t i = 0; i < m_scene.GetTriangles().size(); ++i) {
		m_refstack[i].m_tri_index = i;
		m_refstack[i].m_aabb = m_scene.GetTriangles()[i].GetAABB();
	}

	m_min_overlap_area = m_scene.GetAABB().GetArea() * 1e-5f;

	m_bvh->m_nodes.reserve(m_scene.GetTriangles().size() * 2);

	auto start = std::chrono::steady_clock::now();
	build_node({m_scene.GetAABB(), (uint32_t)m_scene.GetTriangles().size()}, 0);

	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
	spdlog::info("SBVH built with {} nodes in {} ms", m_bvh->m_nodes.size(), duration.count());

	m_bvh->m_nodes.shrink_to_fit();
}
