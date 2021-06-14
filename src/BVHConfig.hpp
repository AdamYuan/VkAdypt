#ifndef BVHCONFIG_HPP
#define BVHCONFIG_HPP

#include <array>
#include <cinttypes>

struct BVHConfig {
	uint32_t m_max_spatial_depth = 50;
	float m_triangle_sah = 0.3f, m_node_sah = 1.0f;
	inline float GetTriangleCost() const { return m_triangle_sah; }
	inline float GetNodeCost() const { return m_node_sah; }
	inline float GetTriangleCost(uint32_t count) const { return m_triangle_sah * count; }
	std::array<uint8_t, 12> ToBytes() const;
	void FromBytes(uint8_t *ptr);
};

#endif
