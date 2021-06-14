#include "BVHConfig.hpp"

#include "Math.hpp"

std::array<uint8_t, 12> BVHConfig::ToBytes() const {
	std::array<uint8_t, 12> ret = {};
	Uint32ToByte4(m_max_spatial_depth, ret.data());
	FloatToByte4(m_triangle_sah, ret.data() + 4);
	FloatToByte4(m_node_sah, ret.data() + 8);
	return ret;
}

void BVHConfig::FromBytes(uint8_t *ptr) {
	m_max_spatial_depth = Byte4ToUint32(ptr);
	m_triangle_sah = Byte4ToFloat(ptr + 4);
	m_node_sah = Byte4ToFloat(ptr + 8);
}
