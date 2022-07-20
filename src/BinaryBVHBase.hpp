// Binary BVH Base
#ifndef ADYPT_BINARY_BVHBASE_HPP
#define ADYPT_BINARY_BVHBASE_HPP

#include <utility>

#include "BVHConfig.hpp"
#include "Scene.hpp"

template <class BVHType> class BinaryBVHBase {
private:
	BVHConfig m_config;
	std::shared_ptr<Scene> m_scene_ptr;

public:
	BinaryBVHBase(const BVHConfig &config, std::shared_ptr<Scene> scene)
	    : m_config{config}, m_scene_ptr{std::move(scene)} {}

	const std::shared_ptr<Scene> &GetScenePtr() const { return m_scene_ptr; }
	const BVHConfig &GetConfig() const { return m_config; }

	bool Empty() const { return ((BVHType *)this)->empty(); }
	bool IsLeaf(uint32_t idx) const { return ((BVHType *)this)->is_leaf(idx); }
	uint32_t GetLeft(uint32_t idx) const { return ((BVHType *)this)->get_left(idx); }
	uint32_t GetRight(uint32_t idx) const { return ((BVHType *)this)->get_right(idx); }
	uint32_t GetTriangleIdx(uint32_t idx) const { return ((BVHType *)this)->get_triangle_idx(idx); }
	const AABB &GetBox(uint32_t idx) const { return ((BVHType *)this)->get_aabb(idx); }

	uint32_t GetLeafCount() const { return ((BVHType *)this)->get_leaf_count(); }
	uint32_t GetNodeCount() const { return ((BVHType *)this)->get_node_count(); }
};

#endif
