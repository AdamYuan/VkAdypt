// Binary BVH Base
#ifndef ADYPT_BINARY_BVHBASE_HPP
#define ADYPT_BINARY_BVHBASE_HPP

#include <utility>

#include "BVHConfig.hpp"
#include "Scene.hpp"

class BinaryBVH {
public:
	struct Node { // a bvh with only 1 primitive per node
		AABB aabb;
		uint32_t tri_idx{};  // triangle index
		uint32_t left_idx{}; // left node index (-1 for leaf nodes) (right node index = current + 1)
	};

private:
	BVHConfig m_config;
	std::shared_ptr<Scene> m_scene_ptr;

	std::vector<Node> m_nodes;
	uint32_t m_leaf_cnt{};

public:
	inline BinaryBVH(const BVHConfig &config, std::shared_ptr<Scene> scene)
	    : m_config{config}, m_scene_ptr{std::move(scene)} {}

	template <typename Builder>
	static std::shared_ptr<BinaryBVH> Build(const BVHConfig &config, const std::shared_ptr<Scene> &scene) {
		std::shared_ptr<BinaryBVH> ret = std::make_shared<BinaryBVH>(config, scene);
		Builder builder{config, *scene};
		builder.Run();
		builder.FetchResult(&ret->m_nodes, &ret->m_leaf_cnt);
		return ret;
	}

	inline const std::shared_ptr<Scene> &GetScenePtr() const { return m_scene_ptr; }
	inline const BVHConfig &GetConfig() const { return m_config; }

	inline bool IsLeaf(uint32_t idx) const { return m_nodes[idx].left_idx == -1; }
	inline uint32_t GetLeft(uint32_t idx) const { return m_nodes[idx].left_idx; }
	inline uint32_t GetRight(uint32_t idx) const { return idx + 1; }
	inline uint32_t GetTriangleIdx(uint32_t idx) const { return m_nodes[idx].tri_idx; }
	inline const AABB &GetAABB(uint32_t idx) const { return m_nodes[idx].aabb; }

	inline uint32_t GetRoot() const { return 0; }
	inline bool Empty() const { return m_nodes.empty(); }
	inline uint32_t GetNodeRange() const { return m_nodes.size(); }
	inline uint32_t GetLeafCount() const { return m_leaf_cnt; }
};

#endif
