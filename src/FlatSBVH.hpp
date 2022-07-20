#ifndef SBVH_HPP
#define SBVH_HPP

#include "BVHConfig.hpp"
#include "BinaryBVHBase.hpp"
#include "Scene.hpp"
#include <memory>
#include <vector>

class FlatSBVH : public BinaryBVHBase<FlatSBVH> {
private:
	struct Node // a bvh with only 1 primitive per node
	{
		AABB m_aabb;
		uint32_t m_tri_idx{};  // triangle index
		uint32_t m_left_idx{}; // left node index (right node index = current + 1)
	};

	std::vector<Node> m_nodes;
	uint32_t m_leaves_cnt{};

public:
	bool empty() const { return m_nodes.empty(); }
	bool is_leaf(uint32_t idx) const { return m_nodes[idx].m_left_idx == -1; }
	uint32_t get_left(uint32_t idx) const { return m_nodes[idx].m_left_idx; }
	static uint32_t get_right(uint32_t idx) { return idx + 1; }
	uint32_t get_triangle_idx(uint32_t idx) const { return m_nodes[idx].m_tri_idx; }
	const AABB &get_aabb(uint32_t idx) const { return m_nodes[idx].m_aabb; }

	uint32_t get_node_count() const { return m_nodes.size(); }
	uint32_t get_leaf_count() const { return m_leaves_cnt; }

public:
	inline FlatSBVH(const BVHConfig &config, const std::shared_ptr<Scene> &scene)
	    : BinaryBVHBase<FlatSBVH>(config, scene) {}
	static std::shared_ptr<BinaryBVHBase<FlatSBVH>> Build(const BVHConfig &config, const std::shared_ptr<Scene> &scene);

	friend class FlatSBVHBuilder;
};

#endif
