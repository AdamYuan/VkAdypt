namespace wide_bvh_detail {

template <class BVHType> void WideBVHBuilder<BVHType>::Run() {
	m_infos.resize(m_bin_bvh.GetNodeRange());
	calculate_cost(m_bin_bvh.GetRoot());
	spdlog::info("WideBVH cost analyzed");

	m_p_wbvh->m_nodes.emplace_back();
	m_p_wbvh->m_tri_indices.reserve((size_t)m_bin_bvh.GetLeafCount());
	create_nodes(m_bin_bvh.GetRoot(), 0);
	spdlog::info("WideBVH built with {} nodes", m_p_wbvh->m_nodes.size());

	m_infos.clear();
	m_infos.shrink_to_fit();
	m_p_wbvh->m_nodes.shrink_to_fit();
}

template <class BVHType>
WideBVHBuilder<BVHType>::WideBVHBuilder(WideBVH *p_wbvh, const BinaryBVHBase<BVHType> &bin_bvh)
    : m_p_wbvh(p_wbvh), m_bin_bvh(bin_bvh), m_config(bin_bvh.GetConfig()) {
	m_p_wbvh->m_nodes.clear();
	m_p_wbvh->m_tri_indices.clear();
}

template <class BVHType>
std::tuple<uint32_t, typename WideBVHBuilder<BVHType>::NodeSAHGroup>
WideBVHBuilder<BVHType>::calculate_cost(BVHIterator node) {
	NodeSAHGroup sah;

	float area = node.GetAABB().GetHalfArea();
	// is leaf, then initialize
	auto node_idx = node.GetIndex();
	if (node.IsLeaf()) {
		for (uint32_t i = 1; i < 8; ++i) {
			sah[i] = m_config.GetTriangleCost() * area;
			m_infos[node_idx][i].m_type = NodeInfo::kLeaf;
		}
		return {1, std::move(sah)};
	}

	auto [left_tri_count, left_sah] = calculate_cost(node.GetLeft());
	auto [right_tri_count, right_sah] = calculate_cost(node.GetRight());
	uint32_t tri_count = left_tri_count + right_tri_count;

	auto &info = m_infos[node_idx];

	{ // for i = 1
		float c_leaf = tri_count <= 3 ? area * m_config.GetTriangleCost(tri_count) : FLT_MAX;

		float c_internal = FLT_MAX;
		{ // calculate c_internal
			float node_sah = area * m_config.GetNodeCost();
			for (uint32_t k = 1; k < 8; ++k) {
				float r = node_sah + left_sah[k] + right_sah[8 - k];
				if (r < c_internal) {
					c_internal = r;
					info[1].m_distribute_0 = k;
					info[1].m_distribute_1 = 8 - k;
				}
			}
		}
		if (c_leaf < c_internal) {
			sah[1] = c_leaf;
			info[1].m_type = NodeInfo::kLeaf;
		} else {
			sah[1] = c_internal;
			info[1].m_type = NodeInfo::kInternal;
		}
	}

	for (uint32_t i = 2; i < 8; ++i) {
		float c_distribute = FLT_MAX;
		for (uint32_t k = 1; k < i; ++k) {
			float r = left_sah[k] + right_sah[i - k];
			if (r < c_distribute) {
				c_distribute = r;
				info[i].m_distribute_0 = k;
				info[i].m_distribute_1 = i - k;
			}
		}
		if (c_distribute < sah[i - 1]) {
			sah[i] = c_distribute;
			info[i].m_type = NodeInfo::kDistribute;
		} else {
			sah[i] = sah[i - 1];
			info[i] = info[i - 1];
		}
	}
	if (node == m_bin_bvh.GetRoot()) {
		spdlog::info("SAH: {}", sah[1]);
	}
	return {tri_count, std::move(sah)};
}

template <class BVHType>
void WideBVHBuilder<BVHType>::fetch_children(BVHIterator node, uint32_t i, uint32_t *out_cnt,
                                             BVHIterator out_nodes[8]) {
	auto node_idx = node.GetIndex();
	const BVHIterator ch[2] = {node.GetLeft(), node.GetRight()};
	uint8_t cdis[2] = {m_infos[node_idx][i].m_distribute_0, m_infos[node_idx][i].m_distribute_1};
	for (uint32_t c = 0; c < 2; ++c) {
		const auto &info = m_infos[ch[c].GetIndex()][cdis[c]];
		if (info.m_type == NodeInfo::kDistribute)
			fetch_children(ch[c], cdis[c], out_cnt, out_nodes);
		else
			out_nodes[(*out_cnt)++] = ch[c];
	}
}

template <class BVHType> uint32_t WideBVHBuilder<BVHType>::fetch_leaves(BVHIterator node) {
	if (node.IsLeaf()) {
		m_p_wbvh->m_tri_indices.push_back(node.GetTriangleIdx());
		return 1;
	}
	return fetch_leaves(node.GetLeft()) + fetch_leaves(node.GetRight());
}

template <class BVHType> void WideBVHBuilder<BVHType>::hungarian(const float mat[8][8], uint32_t n, uint32_t order[8]) {
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

template <class BVHType> void WideBVHBuilder<BVHType>::create_nodes(BVHIterator node, uint32_t wbvh_node_idx) {
#define CUR (m_p_wbvh->m_nodes[wbvh_node_idx])

	BVHIterator ch_arr[8];
	uint32_t ch_cnt = 0;
	fetch_children(node, 1, &ch_cnt, ch_arr);

	const AABB &cur_box = node.GetAABB();
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
	uint32_t ch_slot_arr[8];

	{
		static float ch_cost_mat[8][8];
		glm::vec3 dist;
		for (uint32_t i = 0; i < ch_cnt; ++i)
			for (uint32_t j = 0; j < 8; ++j) {
				dist = ch_arr[i].GetAABB().GetCenter() - node.GetAABB().GetCenter();
				ch_cost_mat[i][j] = ((j & 1u) ? -dist.x : dist.x) + ((j & 2u) ? -dist.y : dist.y) +
				                    ((j & 4u) ? -dist.z : dist.z); // project to diagonal ray
			}
		hungarian(ch_cost_mat, ch_cnt, ch_slot_arr);
	}

	std::optional<BVHIterator> ch_ranked_arr[8]{};
	for (uint32_t i = 0; i < ch_cnt; ++i)
		ch_ranked_arr[ch_slot_arr[i]] = ch_arr[i];

	// set values
	CUR.m_imask = 0;
	CUR.m_child_idx_base = (uint32_t)m_p_wbvh->m_nodes.size();
	CUR.m_tri_idx_base = (uint32_t)m_p_wbvh->m_tri_indices.size();

	for (uint32_t i = 0; i < 8; ++i) {
		if (ch_ranked_arr[i].has_value()) {
			const auto &cur = ch_ranked_arr[i].value();

			glm::uvec3 qlow = glm::floor((cur.GetAABB().min - cur_box.min) / cell);
			glm::uvec3 qhigh = glm::ceil((cur.GetAABB().max - cur_box.min) / cell);
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

			if (m_infos[cur.GetIndex()][1].m_type == NodeInfo::kLeaf) {
				uint32_t tidx = m_p_wbvh->m_tri_indices.size() - CUR.m_tri_idx_base;
				uint32_t tri_cnt = fetch_leaves(cur);
				// bbbindex
				constexpr uint32_t kLeafMetaMap[4] = {0u, 0b00100000u, 0b01100000u, 0b11100000u};
				CUR.m_meta[i] = kLeafMetaMap[tri_cnt] | tidx;
			} else if (m_infos[cur.GetIndex()][1].m_type == NodeInfo::kInternal) {
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
		if (m_infos[ch_arr[i].GetIndex()][1].m_type == NodeInfo::kInternal)
			create_nodes(ch_arr[i], CUR.m_child_idx_base + (CUR.m_meta[ch_slot_arr[i]] & 0x1fu) - 24u);
#undef CUR
}

} // namespace wide_bvh_detail