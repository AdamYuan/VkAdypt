#ifndef SBVH_HPP
#define SBVH_HPP

#include "BVHConfig.hpp"
#include "FlatBinaryBVH.hpp"
#include "Scene.hpp"
#include <memory>
#include <vector>

class SBVHBuilder {
public:
	using BVHType = FlatBinaryBVH;

private:
	static constexpr uint32_t kSpatialBinNum = 32;

	struct Reference {
		AABB m_aabb;
		uint32_t m_tri_index{};
	};
	struct NodeSpec {
		AABB m_aabb;
		uint32_t m_ref_num{};
	};
	struct ObjectSplit {
		AABB left_aabb, right_aabb;
		uint32_t dim{};
		float pos{}, sah{FLT_MAX};
	};
	struct SpatialSplit {
		uint32_t m_dim;
		float m_pos, m_sah;
	};
	struct SpatialBin {
		AABB m_aabb;
		uint32_t m_in{}, m_out{};
	};

	FlatBinaryBVH &m_bvh;
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

	template <uint32_t DIM> inline void _find_object_split_dim(const NodeSpec &t_spec, ObjectSplit *t_os);
	inline void find_object_split(const NodeSpec &t_spec, ObjectSplit *t_os);
	inline void perform_object_split(const NodeSpec &t_spec, const ObjectSplit &t_os, NodeSpec *t_left,
	                                 NodeSpec *t_right);
	inline void split_reference(const Reference &t_ref, uint32_t t_dim, float t_pos, Reference *t_left,
	                            Reference *t_right);
	template <uint32_t DIM> inline void _find_spatial_split_dim(const NodeSpec &t_spec, SpatialSplit *t_ss);
	inline void find_spatial_split(const NodeSpec &t_spec, SpatialSplit *t_ss);
	inline void perform_spatial_split(const NodeSpec &t_spec, const SpatialSplit &t_ss, NodeSpec *t_left,
	                                  NodeSpec *t_right);
	uint32_t build_node(const NodeSpec &t_spec, uint32_t t_depth);
	inline uint32_t push_node() {
		m_bvh.m_nodes.emplace_back();
		return m_bvh.m_nodes.size() - 1;
	}

public:
	inline explicit SBVHBuilder(FlatBinaryBVH *p_bvh)
	    : m_bvh(*p_bvh), m_scene(*p_bvh->GetScenePtr()),
	      m_config(p_bvh->GetConfig()), m_min_overlap_area{p_bvh->GetScenePtr()->GetAABB().GetHalfArea() * 1e-5f} {}
	void Run();
};

#endif
