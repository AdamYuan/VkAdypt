#include "WideBVH.hpp"
#include <cstring>
#include <fstream>
#include <spdlog/spdlog.h>

class WideBVHBuilder {
private:
	const BinaryBVH &m_bin_bvh;
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
	// out_cnt  describes the number of children
	// out_idx  stores the children index
	void fetch_children(uint32_t node_idx, uint32_t i, uint32_t *out_cnt, uint32_t out_node_idx[8]);
	//
	uint32_t fetch_leaves(uint32_t node_idx);
	// hungarian algorithm to solve the min-assignment problem
	static void hungarian(const float mat[8][8], uint32_t n, uint32_t order[8]);
	void create_nodes(uint32_t wbvh_node_idx, uint32_t sbvh_node_idx);

public:
	WideBVHBuilder(WideBVH *p_wbvh, const BinaryBVH &bin_bvh);
	void Run();
};

std::shared_ptr<WideBVH> WideBVH::Build(const std::shared_ptr<BinaryBVH> &bin_bvh) {
	std::shared_ptr<WideBVH> ret = std::make_shared<WideBVH>(bin_bvh->GetConfig(), bin_bvh->GetScenePtr());
	WideBVHBuilder builder{ret.get(), *bin_bvh};
	builder.Run();
	return ret;
}

bool WideBVH::SaveToFile(const char *filename) {
	std::ofstream os{filename, std::ios::binary};
	if (!os.is_open())
		return false;

	os.write(kVersionStr, strlen(kVersionStr) + 1);
	os.write((char *)&m_config, sizeof(BVHConfig));
	auto tri_indeices_size = (uint32_t)m_tri_indices.size();
	os.write((char *)&tri_indeices_size, sizeof(uint32_t));
	os.write((char *)m_tri_indices.data(), m_tri_indices.size() * sizeof(int32_t));
	os.write((char *)m_nodes.data(), m_nodes.size() * sizeof(Node));

	return true;
}

/*std::shared_ptr<WideBVH> WideBVH::CreateFromFile(const char *filename, const BVHConfig &expected_config) {
    std::shared_ptr<WideBVH> ret = std::make_shared<WideBVH>();
    ret->m_config = expected_config;

    std::ifstream is{filename, std::ios::binary};
    if (!is.is_open())
        return nullptr;

    std::vector<char> buffer{std::istreambuf_iterator<char>(is), std::istreambuf_iterator<char>()};
    char *ptr = buffer.data(), *end = buffer.data() + buffer.size();

    // compare cwbvh version
    if (strcmp(ptr, kVersionStr) != 0)
        return nullptr;

    ptr += strlen(kVersionStr) + 1;

    auto config = *(BVHConfig *)ptr;
    ptr += sizeof(BVHConfig);

    if (config.m_node_sah != expected_config.m_node_sah || config.m_triangle_sah != expected_config.m_triangle_sah ||
        config.m_max_spatial_depth != expected_config.m_max_spatial_depth)
        return nullptr;

    uint32_t tri_indices_size = *(uint32_t *)ptr;
    ptr += sizeof(uint32_t);

    // read triangle indices
    for (int i = 0; i < tri_indices_size && ptr < end; ++i) {
        ret->m_tri_indices.push_back(*(int32_t *)ptr);
        ptr += sizeof(int32_t);
    }

    // read node
    while (ptr < end) {
        ret->m_nodes.push_back(*(WideBVHNode *)ptr);
        ptr += sizeof(WideBVHNode);
    }

    return ret;
}*/

void WideBVHBuilder::Run() {
	m_costs.resize(m_bin_bvh.GetNodeRange());
	calculate_cost(m_bin_bvh.GetRoot());
	spdlog::info("WideBVH cost analyzed");

	m_p_wbvh->m_nodes.emplace_back();
	m_p_wbvh->m_tri_indices.reserve((size_t)m_bin_bvh.GetLeafCount());
	create_nodes(0, m_bin_bvh.GetRoot());
	spdlog::info("WideBVH built with {} nodes", m_p_wbvh->m_nodes.size());

	m_p_wbvh->m_nodes.shrink_to_fit();
	m_costs.clear();
	m_costs.shrink_to_fit();
}

WideBVHBuilder::WideBVHBuilder(WideBVH *p_wbvh, const BinaryBVH &bin_bvh)
    : m_p_wbvh(p_wbvh), m_bin_bvh(bin_bvh), m_config(bin_bvh.GetConfig()) {
	m_p_wbvh->m_nodes.clear();
	m_p_wbvh->m_tri_indices.clear();
}

uint32_t WideBVHBuilder::calculate_cost(uint32_t node_idx) {
	float area = m_bin_bvh.GetAABB(node_idx).GetArea();
	// is leaf, then initialize
	if (m_bin_bvh.IsLeaf(node_idx)) {
		for (uint32_t i = 1; i <= 7; ++i) {
			m_costs[node_idx][i].m_sah = m_config.GetTriangleCost() * area;
			m_costs[node_idx][i].m_type = NodeCost::kLeaf;
		}
		return 1;
	}

	uint32_t lidx = m_bin_bvh.GetLeft(node_idx), ridx = m_bin_bvh.GetRight(node_idx);
	uint32_t tri_count = calculate_cost(ridx) + calculate_cost(lidx);

	auto &dp = m_costs[node_idx];

	{ // for i = 1
		float c_leaf = tri_count <= 3 ? area * m_config.GetTriangleCost(tri_count) : FLT_MAX;

		float c_internal = FLT_MAX;
		{ // calculate c_internal
			float node_sah = area * m_config.GetNodeCost();
			for (uint32_t k = 1; k < 8; ++k) {
				float r = node_sah + m_costs[lidx][k].m_sah + m_costs[ridx][8 - k].m_sah;
				if (r < c_internal) {
					c_internal = r;
					dp[1].m_distribute[0] = k;
					dp[1].m_distribute[1] = 8 - k;
				}
			}
		}
		if (c_leaf < c_internal) {
			dp[1].m_sah = c_leaf;
			dp[1].m_type = NodeCost::kLeaf;
		} else {
			dp[1].m_sah = c_internal;
			dp[1].m_type = NodeCost::kInternal;
		}
	}

	for (uint32_t i = 2; i <= 7; ++i) {
		float c_distribute = FLT_MAX;
		for (uint32_t k = 1; k < i; ++k) {
			float r = m_costs[lidx][k].m_sah + m_costs[ridx][i - k].m_sah;
			if (r < c_distribute) {
				c_distribute = r;
				dp[i].m_distribute[0] = k;
				dp[i].m_distribute[1] = i - k;
			}
		}
		if (c_distribute < dp[i - 1].m_sah) {
			dp[i].m_sah = c_distribute;
			dp[i].m_type = NodeCost::kDistribute;
		} else
			dp[i] = dp[i - 1];
	}

	return tri_count;
}

void WideBVHBuilder::fetch_children(uint32_t node_idx, uint32_t i, uint32_t *out_cnt, uint32_t out_node_idx[8]) {
	uint32_t cidx[2] = {m_bin_bvh.GetLeft(node_idx), m_bin_bvh.GetRight(node_idx)};
	uint32_t cdis[2] = {m_costs[node_idx][i].m_distribute[0], m_costs[node_idx][i].m_distribute[1]};
	for (uint32_t c = 0; c < 2; ++c) {
		const auto &info = m_costs[cidx[c]][cdis[c]];
		if (info.m_type == NodeCost::kDistribute)
			fetch_children(cidx[c], cdis[c], out_cnt, out_node_idx);
		else
			out_node_idx[(*out_cnt)++] = cidx[c];
	}
}

uint32_t WideBVHBuilder::fetch_leaves(uint32_t node_idx) {
	if (m_bin_bvh.IsLeaf(node_idx)) {
		m_p_wbvh->m_tri_indices.push_back(m_bin_bvh.GetTriangleIdx(node_idx));
		return 1;
	}
	return fetch_leaves(m_bin_bvh.GetRight(node_idx)) + fetch_leaves(m_bin_bvh.GetLeft(node_idx));
}

void WideBVHBuilder::hungarian(const float mat[8][8], uint32_t n, uint32_t order[8]) {
#define INF 1e12f
	static uint32_t p[9], way[9];
	static float u[9], v[9], minv[9];
	static bool used[9];

	std::fill(u, u + n + 1, 0.0f);
	std::fill(v, v + 9, 0.0f);
	std::fill(way, way + 9, 0);
	std::fill(p, p + 9, 0);

	for (uint32_t i = 1; i <= n; ++i) {
		p[0] = i;
		uint32_t j0 = 0;
		std::fill(minv, minv + 9, INF);
		std::fill(used, used + 9, false);
		do {
			used[j0] = true;
			uint32_t i0 = p[j0], j1{};
			float delta = INF;
			for (uint32_t j = 1; j <= 8; ++j)
				if (!used[j]) {
					float cur = mat[i0 - 1][j - 1] - u[i0] - v[j];
					if (cur < minv[j])
						minv[j] = cur, way[j] = j0;
					if (minv[j] < delta)
						delta = minv[j], j1 = j;
				}
			for (uint32_t j = 0; j <= 8; ++j)
				if (used[j])
					u[p[j]] += delta, v[j] -= delta;
				else
					minv[j] -= delta;
			j0 = j1;
		} while (p[j0] != 0);
		do {
			uint32_t j1 = way[j0];
			p[j0] = p[j1];
			j0 = j1;
		} while (j0);
	}
	for (uint32_t i = 1; i <= 8; ++i) {
		if (p[i])
			order[p[i] - 1] = i - 1;
	}
#undef INF
}

void WideBVHBuilder::create_nodes(uint32_t wbvh_node_idx, uint32_t sbvh_node_idx) {
#define CUR (m_p_wbvh->m_nodes[wbvh_node_idx])

	uint32_t ch_idx_arr[8], ch_cnt = 0;
	fetch_children(sbvh_node_idx, 1, &ch_cnt, ch_idx_arr);

	const AABB &cur_box = m_bin_bvh.GetAABB(sbvh_node_idx);
	glm::vec3 cell; // cell size
	{
		// fetch lo position
		CUR.m_px = cur_box.min.x;
		CUR.m_py = cur_box.min.y;
		CUR.m_pz = cur_box.min.z;

		constexpr auto kBase = float(1.0 / double((1 << 8) - 1));
		cell = (cur_box.max - cur_box.min) * kBase;

		CUR.m_ex = cell.x == 0.0f ? 0u : (uint8_t)(127 + (int32_t)std::ceil(std::log2(cell.x)));
		CUR.m_ey = cell.y == 0.0f ? 0u : (uint8_t)(127 + (int32_t)std::ceil(std::log2(cell.y)));
		CUR.m_ez = cell.z == 0.0f ? 0u : (uint8_t)(127 + (int32_t)std::ceil(std::log2(cell.z)));

		cell.x = glm::uintBitsToFloat((uint32_t)(CUR.m_ex) << 23u);
		cell.y = glm::uintBitsToFloat((uint32_t)(CUR.m_ey) << 23u);
		cell.z = glm::uintBitsToFloat((uint32_t)(CUR.m_ez) << 23u);
	}

	// ordering the children with hungarian assignment algorithm
	std::array<uint32_t, 8> ch_slot_arr{};

	{
		static float ch_cost_mat[8][8];
		glm::vec3 dist;
		for (uint32_t i = 0; i < ch_cnt; ++i)
			for (uint32_t j = 0; j < 8; ++j) {
				dist = m_bin_bvh.GetAABB(ch_idx_arr[i]).GetCenter() - m_bin_bvh.GetAABB(sbvh_node_idx).GetCenter();
				ch_cost_mat[i][j] = ((j & 1u) ? -dist.x : dist.x) + ((j & 2u) ? -dist.y : dist.y) +
				                    ((j & 4u) ? -dist.z : dist.z); // project to diagonal ray
			}
		hungarian(ch_cost_mat, ch_cnt, ch_slot_arr.data());
	}

	uint32_t ch_ranked_idx_arr[8];
	std::fill(ch_ranked_idx_arr, ch_ranked_idx_arr + 8, UINT32_MAX);
	for (uint32_t i = 0; i < ch_cnt; ++i)
		ch_ranked_idx_arr[ch_slot_arr[i]] = ch_idx_arr[i];

	// set values
	CUR.m_imask = 0;
	CUR.m_child_idx_base = (uint32_t)m_p_wbvh->m_nodes.size();
	CUR.m_tri_idx_base = (uint32_t)m_p_wbvh->m_tri_indices.size();

	for (uint32_t i = 0; i < 8; ++i) {
		uint32_t sidx = ch_ranked_idx_arr[i];
		if (~sidx) {
			glm::uvec3 qlow = glm::floor((m_bin_bvh.GetAABB(sidx).min - cur_box.min) / cell);
			glm::uvec3 qhigh = glm::ceil((m_bin_bvh.GetAABB(sidx).max - cur_box.min) / cell);
			// TODO: cast NaN to uint ?
			qlow = glm::min(qlow, glm::uvec3(UINT8_MAX));
			qhigh = glm::min(qhigh, glm::uvec3(UINT8_MAX));
			qlow = glm::max(qlow, glm::uvec3(0));
			qhigh = glm::max(qhigh, glm::uvec3(0));

			CUR.m_qlox[i] = (uint8_t)qlow.x;
			CUR.m_qloy[i] = (uint8_t)qlow.y;
			CUR.m_qloz[i] = (uint8_t)qlow.z;
			CUR.m_qhix[i] = (uint8_t)qhigh.x;
			CUR.m_qhiy[i] = (uint8_t)qhigh.y;
			CUR.m_qhiz[i] = (uint8_t)qhigh.z;

			if (m_costs[sidx][1].m_type == NodeCost::kLeaf) {
				uint32_t tidx = m_p_wbvh->m_tri_indices.size() - CUR.m_tri_idx_base;
				uint32_t tri_cnt = fetch_leaves(sidx);
				// bbbindex
				constexpr uint32_t kLeafMetaMap[4] = {0u, 0b00100000u, 0b01100000u, 0b11100000u};
				CUR.m_meta[i] = kLeafMetaMap[tri_cnt] | tidx;
			} else if (m_costs[sidx][1].m_type == NodeCost::kInternal) {
				uint32_t widx = m_p_wbvh->m_nodes.size() - CUR.m_child_idx_base;
				m_p_wbvh->m_nodes.emplace_back();
				// 001index
				CUR.m_meta[i] = (1 << 5u) | (widx + 24u);
				// mark as internal
				CUR.m_imask |= (1u << widx);
			}
		} else
			CUR.m_meta[i] = 0u;
	}

	for (uint32_t i = 0; i < ch_cnt; ++i)
		if (m_costs[ch_idx_arr[i]][1].m_type == NodeCost::kInternal)
			create_nodes(CUR.m_child_idx_base + (CUR.m_meta[ch_slot_arr[i]] & 0x1fu) - 24u, ch_idx_arr[i]);
#undef CUR
}
