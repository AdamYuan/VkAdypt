#ifndef FLAT_BVH_HPP
#define FLAT_BVH_HPP

#include "BVHConfig.hpp"
#include "BinaryBVHBase.hpp"
#include "Scene.hpp"
#include <memory>
#include <utility>
#include <vector>

class FlatBinaryBVH : public BinaryBVHBase<FlatBinaryBVH> {
public:
	using Iterator = uint32_t;

private:
	struct Node { // a bvh with only 1 primitive per node
		AABB aabb;
		uint32_t tri_idx{};  // triangle index
		uint32_t left_idx{}; // left node index (-1 for leaf nodes) (right node index = current + 1)
	};

	std::vector<Node> m_nodes;
	uint32_t m_leaf_cnt{};

public:
	inline FlatBinaryBVH(const BVHConfig &config, std::shared_ptr<Scene> scene)
	    : BinaryBVHBase<FlatBinaryBVH>(config, std::move(scene)) {}
	~FlatBinaryBVH() override = default;

	inline bool is_leaf(uint32_t idx) const { return m_nodes[idx].left_idx == -1; }
	static inline uint32_t get_index(uint32_t idx) { return idx; }
	inline uint32_t get_left(uint32_t idx) const { return m_nodes[idx].left_idx; }
	inline uint32_t get_right(uint32_t idx) const { return idx + 1; }
	inline uint32_t get_triangle_idx(uint32_t idx) const { return m_nodes[idx].tri_idx; }
	inline const AABB &get_aabb(uint32_t idx) const { return m_nodes[idx].aabb; }

	inline uint32_t get_root() const { return 0; }
	inline bool empty() const { return m_nodes.empty(); }

	inline uint32_t get_node_count() const { return m_nodes.size(); }
	inline uint32_t get_leaf_count() const { return m_leaf_cnt; }

	friend class SBVHBuilder;
};

#endif
