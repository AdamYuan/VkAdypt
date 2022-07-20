#ifndef ADYPT_WIDEBVH_HPP
#define ADYPT_WIDEBVH_HPP

#include "BinaryBVHBase.hpp"
#include <cinttypes>
#include <memory>
#include <vector>

#include "WideBVHDecl.inl"

class WideBVH {
public:
	// a compressed 80 byte bvh node
	struct Node {
		float m_px, m_py, m_pz;
		uint8_t m_ex, m_ey, m_ez, m_imask;
		uint32_t m_child_idx_base; // child node base index
		uint32_t m_tri_idx_base;   // triangle base index
		uint8_t m_meta[8];
		uint8_t m_qlox[8];
		uint8_t m_qloy[8];
		uint8_t m_qloz[8];
		uint8_t m_qhix[8];
		uint8_t m_qhiy[8];
		uint8_t m_qhiz[8];
	};

private:
	BVHConfig m_config;
	std::shared_ptr<Scene> m_scene_ptr;

	// for saving file
	static constexpr const char *kVersionStr = "CWBVH_1.0";
	std::vector<Node> m_nodes;
	std::vector<uint32_t> m_tri_indices;

public:
	WideBVH(const BVHConfig &config, std::shared_ptr<Scene> scene) : m_config{config}, m_scene_ptr{std::move(scene)} {}

	template <class BVHType>
	static std::shared_ptr<WideBVH> Build(const std::shared_ptr<BinaryBVHBase<BVHType>> &bin_bvh) {
		std::shared_ptr<WideBVH> ret = std::make_shared<WideBVH>(bin_bvh->GetConfig(), bin_bvh->GetScenePtr());
		wide_bvh_detail::WideBVHBuilder<BVHType> builder{ret.get(), *bin_bvh};
		builder.Run();
		return ret;
	}
	// static std::shared_ptr<WideBVH> CreateFromFile(const char *filename, const BVHConfig &expected_config);

	bool SaveToFile(const char *filename);

	const std::shared_ptr<Scene> &GetScenePtr() const { return m_scene_ptr; }

	const BVHConfig &GetConfig() const { return m_config; }

	const std::vector<Node> &GetNodes() const { return m_nodes; }
	const std::vector<uint32_t> &GetTriIndices() const { return m_tri_indices; }

	template <class BVHType> friend class wide_bvh_detail::WideBVHBuilder;
};

#include "WideBVHImpl.inl"

#endif
