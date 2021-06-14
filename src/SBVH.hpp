#ifndef SBVH_HPP
#define SBVH_HPP

#include "Scene.hpp"
#include "BVHConfig.hpp"
#include <memory>
#include <vector>

struct SBVHNode // a bvh with only 1 primitive per node
{
	AABB m_aabb;
	uint32_t m_tri_idx;  // triangle index
	uint32_t m_left_idx; // left node index (right node index = current + 1)
};

class SBVH {
private:
	std::shared_ptr<Scene> m_scene_ptr;

	std::vector<SBVHNode> m_nodes;
	uint32_t m_leaves_cnt;

	BVHConfig m_config;

public:
	SBVH() = default;

	static std::shared_ptr<SBVH> Build(const BVHConfig &config, const std::shared_ptr<Scene> &scene);

	const std::shared_ptr<Scene> &GetScenePtr() const { return m_scene_ptr; }

	const BVHConfig &GetConfig() const { return m_config; }
	const std::vector<SBVHNode> &GetNodes() const { return m_nodes; }
	bool Empty() const { return m_nodes.empty(); }
	bool IsLeaf(uint32_t idx) const { return m_nodes[idx].m_left_idx == -1; }
	uint32_t GetLeft(uint32_t idx) const { return m_nodes[idx].m_left_idx; }
	uint32_t GetRight(uint32_t idx) const { return idx + 1; }
	uint32_t GetTriIdx(uint32_t idx) const { return m_nodes[idx].m_tri_idx; }
	const AABB &GetBox(uint32_t idx) const { return m_nodes[idx].m_aabb; }

	uint32_t GetLeafCount() const { return m_leaves_cnt; }
	friend class SBVHBuilder;
};

#endif
