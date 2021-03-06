#include <algorithm>
#include <optional>
#include <spdlog/spdlog.h>

class WideBVH;

namespace wide_bvh_detail {

template <class BVHType> class WideBVHBuilder {
private:
	using BVHIterator = typename BinaryBVHBase<BVHType>::Iterator;
	const BinaryBVHBase<BVHType> &m_bin_bvh;
	const BVHConfig &m_config;
	WideBVH *m_p_wbvh;

	struct NodeInfo {
		enum Type { kInternal = 0, kLeaf, kDistribute };
		uint8_t m_type : 2;
		uint8_t m_distribute_0 : 3;
		uint8_t m_distribute_1 : 3;
	};
	static_assert(sizeof(NodeInfo) == 1);
	struct NodeInfoGroup {
		std::array<NodeInfo, 7> arr;
		NodeInfo &operator[](uint32_t i) { return arr[i - 1]; }
	};
	static_assert(sizeof(NodeInfoGroup) == 7);

	struct NodeSAHGroup {
		std::array<float, 7> arr;
		float &operator[](uint32_t i) { return arr[i - 1]; }
	};

	std::vector<NodeInfoGroup> m_infos;
	// return triangle count and distribution SAHs
	std::tuple<uint32_t, NodeSAHGroup> calculate_cost(BVHIterator node);
	//{node_idx, i} are the two dimensions of dp array
	// out_size describes the number of children
	// out_idx  stores the children index
	void fetch_children(BVHIterator node, uint32_t i, uint32_t *out_size, BVHIterator out_nodes[8]);
	//
	uint32_t fetch_leaves(BVHIterator node);
	// hungarian algorithm to solve the min-assignment problem
	void hungarian(const float mat[8][8], uint32_t n, uint32_t order[8]);
	void create_nodes(BVHIterator node, uint32_t wbvh_node_idx);

public:
	WideBVHBuilder(WideBVH *p_wbvh, const BinaryBVHBase<BVHType> &bin_bvh);
	void Run();
};

} // namespace wide_bvh_detail
