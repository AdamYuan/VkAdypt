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
	inline BinaryBVHBase(const BVHConfig &config, std::shared_ptr<Scene> scene)
	    : m_config{config}, m_scene_ptr{std::move(scene)} {}

	inline const std::shared_ptr<Scene> &GetScenePtr() const { return m_scene_ptr; }
	inline const BVHConfig &GetConfig() const { return m_config; }

	inline bool Empty() const { return ((BVHType *)this)->empty(); }
	inline bool IsLeaf(uint32_t idx) const { return ((BVHType *)this)->is_leaf(idx); }
	inline uint32_t GetLeft(uint32_t idx) const { return ((BVHType *)this)->get_left(idx); }
	inline uint32_t GetRight(uint32_t idx) const { return ((BVHType *)this)->get_right(idx); }
	inline uint32_t GetTriangleIdx(uint32_t idx) const { return ((BVHType *)this)->get_triangle_idx(idx); }
	inline const AABB &GetAABB(uint32_t idx) const { return ((BVHType *)this)->get_aabb(idx); }

	inline uint32_t GetNodeRange() const { return ((BVHType *)this)->get_node_range(); }
	inline uint32_t GetLeafCount() const { return ((BVHType *)this)->get_leaf_count(); }
};

#endif
