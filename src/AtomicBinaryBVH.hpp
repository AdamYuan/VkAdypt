#ifndef ADYPT_ATOMIC_BINARY_BVH_HPP
#define ADYPT_ATOMIC_BINARY_BVH_HPP

#include "AtomicAllocator.hpp"
#include "BinaryBVHBase.hpp"

class AtomicBinaryBVH : public BinaryBVHBase<AtomicBinaryBVH> {
public:
	struct Node {
		AABB aabb;
		uint32_t left{};
		union {
			uint32_t right{};
			uint32_t tri_idx;
		};
	};
	static_assert(sizeof(Node) == 32);

	using Iterator = uint32_t;

private:
	AtomicAllocator<Node> m_node_pool;

	uint32_t m_leaf_cnt{};

public:
	inline bool empty() const { return m_leaf_cnt == 0; }
	inline bool is_leaf(uint32_t x) const { return m_node_pool[x].left == 0; }
	inline uint32_t get_index(uint32_t x) const { return x; }
	inline uint32_t get_left(uint32_t x) const { return m_node_pool[x].left; }
	inline uint32_t get_right(uint32_t x) const { return m_node_pool[x].right; }
	inline uint32_t get_triangle_idx(uint32_t x) const { return m_node_pool[x].tri_idx; }
	inline const AABB &get_aabb(uint32_t x) const { return m_node_pool[x].aabb; }

	inline uint32_t get_root() const { return 0; }

	inline uint32_t get_node_range() const { return m_node_pool.GetRange(); }
	inline uint32_t get_leaf_count() const { return m_leaf_cnt; }

public:
	inline AtomicBinaryBVH(const BVHConfig &config, const std::shared_ptr<Scene> &scene)
	    : BinaryBVHBase<AtomicBinaryBVH>(config, scene) {}
	~AtomicBinaryBVH() override = default;

	friend class ParallelSBVHBuilder;
};

#endif
