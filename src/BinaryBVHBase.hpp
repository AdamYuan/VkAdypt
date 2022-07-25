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
	class Iterator {
	private:
		const BVHType *m_p_bvh{nullptr};
		typename BVHType::Iterator m_iterator;

	public:
		inline Iterator() = default;
		inline explicit Iterator(const BVHType *p_bvh) : m_p_bvh{p_bvh}, m_iterator{p_bvh->get_root()} {}
		inline bool operator==(const Iterator &r) const { return m_p_bvh == r.m_p_bvh && m_iterator == r.m_iterator; }
		inline bool operator!=(const Iterator &r) const { return m_p_bvh != r.m_p_bvh || m_iterator != r.m_iterator; }
		inline Iterator(const BVHType *p_bvh, const typename BVHType::Iterator &iterator)
		    : m_p_bvh{p_bvh}, m_iterator{iterator} {}
		inline uint32_t GetIndex() const { return m_p_bvh->get_index(m_iterator); }
		inline Iterator GetLeft() const { return Iterator{m_p_bvh, m_p_bvh->get_left(m_iterator)}; }
		inline Iterator GetRight() const { return Iterator{m_p_bvh, m_p_bvh->get_right(m_iterator)}; }
		inline bool IsLeaf() const { return m_p_bvh->is_leaf(m_iterator); }
		inline uint32_t GetTriangleIdx() const { return m_p_bvh->get_triangle_idx(m_iterator); }
		inline const AABB &GetAABB() const { return m_p_bvh->get_aabb(m_iterator); }
	};

	BinaryBVHBase(const BVHConfig &config, std::shared_ptr<Scene> scene)
	    : m_config{config}, m_scene_ptr{std::move(scene)} {}
	virtual ~BinaryBVHBase() = default;

	template <typename Builder, typename = std::enable_if_t<std::is_same_v<BVHType, typename Builder::BVHType>>>
	static std::shared_ptr<BinaryBVHBase<BVHType>> Build(const BVHConfig &config, const std::shared_ptr<Scene> &scene) {
		std::shared_ptr<BVHType> ret = std::make_shared<BVHType>(config, scene);
		Builder builder{ret.get()};
		builder.Run();
		return ret;
	}

	inline const std::shared_ptr<Scene> &GetScenePtr() const { return m_scene_ptr; }
	inline const BVHConfig &GetConfig() const { return m_config; }

	inline bool Empty() const { return ((BVHType *)this)->empty(); }
	inline Iterator GetRoot() const { return Iterator{(BVHType *)this}; }

	uint32_t GetLeafCount() const { return ((BVHType *)this)->get_leaf_count(); }
	uint32_t GetNodeRange() const { return ((BVHType *)this)->get_node_range(); }
};

#endif
