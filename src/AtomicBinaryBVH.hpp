#ifndef ADYPT_ATOMIC_BINARY_BVH_HPP
#define ADYPT_ATOMIC_BINARY_BVH_HPP

#include "AtomicAllocator.hpp"
#include "BinaryBVHBase.hpp"

class AtomicBinaryBVH : public BinaryBVHBase<AtomicBinaryBVH> {
public:
	struct Node {
		AABB aabb;
		Node *left{}, *right{};
		uint32_t tri_idx{}, node_idx{};
	};
	using Iterator = const Node *;

private:
	static constexpr uint32_t kAtomicAllocatorChunk = 256;

	Node m_root{};
	AtomicAllocator<Node, kAtomicAllocatorChunk> m_node_pool;

	uint32_t m_node_cnt{}, m_leaf_cnt{};

public:
	inline bool empty() const { return m_leaf_cnt == 0; }
	static inline bool is_leaf(const Node *node) { return node->left == nullptr; }
	static inline uint32_t get_index(const Node *node) { return node->node_idx; }
	static inline const Node *get_left(const Node *node) { return node->left; }
	static inline const Node *get_right(const Node *node) { return node->right; }
	static inline uint32_t get_triangle_idx(const Node *node) { return node->tri_idx; }
	static inline const AABB &get_aabb(const Node *node) { return node->aabb; }

	inline const Node *get_root() const { return &m_root; }

	inline uint32_t get_node_count() const { return m_node_cnt; }
	inline uint32_t get_leaf_count() const { return m_leaf_cnt; }

public:
	inline AtomicBinaryBVH(const BVHConfig &config, const std::shared_ptr<Scene> &scene)
	    : BinaryBVHBase<AtomicBinaryBVH>(config, scene) {}
	~AtomicBinaryBVH() override = default;

	friend class ParallelSBVHBuilder;
};

#endif
