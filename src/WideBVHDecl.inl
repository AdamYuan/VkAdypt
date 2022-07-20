#include <algorithm>
#include <spdlog/spdlog.h>

class WideBVH;

namespace wide_bvh_detail {

template <class BVHType> class WideBVHBuilder {
private:
	const BinaryBVHBase<BVHType> &m_bin_bvh;
	const BVHConfig &m_config;
	WideBVH *m_p_wbvh;

	struct NodeCost {
		float m_sah;
		enum Type { kInternal = 0, kLeaf, kDistribute };
		uint32_t m_type;
		uint32_t m_distribute[2];
	};
	struct NodeCostGroup {
		NodeCost m_arr[7];
		NodeCost &operator[](uint32_t i) { return m_arr[i - 1]; }
	};

	std::vector<NodeCostGroup> m_costs;
	uint32_t calculate_cost(uint32_t node_idx);
	//{node_idx, i} are the two dimensions of dp array
	// out_size describes the number of children
	// out_idx  stores the children index
	void fetch_children(uint32_t node_idx, uint32_t i, uint32_t *out_size, uint32_t out_node_idx[8]);
	//
	uint32_t fetch_leaves(uint32_t node_idx);
	// hungarian algorithm to solve the min-assignment problem
	void hungarian(const float mat[8][8], uint32_t n, uint32_t order[8]);
	void create_nodes(uint32_t wbvh_node_idx, uint32_t sbvh_node_idx);

public:
	WideBVHBuilder(WideBVH *p_wbvh, const BinaryBVHBase<BVHType> &bin_bvh);
	void Run();
};

} // namespace wide_bvh_detail
