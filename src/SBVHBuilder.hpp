#ifndef SBVH_HPP
#define SBVH_HPP

#include "BVHConfig.hpp"
#include "BinaryBVH.hpp"
#include "Scene.hpp"
#include <memory>
#include <vector>

class SBVHBuilder {
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
		AABB m_left_aabb, m_right_aabb;
		uint32_t m_dim{}, m_left_num{};
		float m_sah{};
	};
	struct SpatialSplit {
		uint32_t m_dim;
		float m_pos, m_sah;
	};
	struct SpatialBin {
		AABB m_aabb;
		uint32_t m_in{}, m_out{};
	};

	std::vector<BinaryBVH::Node> m_nodes;
	uint32_t m_leaf_cnt{};
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
		m_nodes.emplace_back();
		return m_nodes.size() - 1;
	}

public:
	inline SBVHBuilder(const BVHConfig &config, const Scene &scene)
	    : m_scene(scene), m_config(config), m_min_overlap_area{scene.GetAABB().GetArea() * 1e-5f} {}
	void Run();
	inline void FetchResult(std::vector<BinaryBVH::Node> *p_nodes, uint32_t *p_leaf_count) {
		*p_nodes = std::move(m_nodes);
		*p_leaf_count = m_leaf_cnt;
	}
};

#endif
