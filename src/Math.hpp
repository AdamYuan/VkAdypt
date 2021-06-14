#ifndef MATH_HPP
#define MATH_HPP

#include <glm/glm.hpp>

inline glm::vec2 SpheremapEncode(const glm::vec3 &n) { return n.xy() / glm::sqrt(n.z * 8 + 8) + 0.5f; }
inline glm::vec3 SpheremapDecode(const glm::vec2 &enc) {
	glm::vec4 nn = glm::vec4(enc, 0, 0) * glm::vec4(2, 2, 0, 0) + glm::vec4(-1, -1, 1, -1);
	nn.z = glm::dot(nn.xyz(), -nn.xyw());
	float sn = glm::sqrt(nn.z);
	nn.x *= sn;
	nn.y *= sn;
	return nn.xyz() * 2.0f + glm::vec3(0, 0, -1);
}

inline uint32_t Byte4ToUint32(const uint8_t *byte4) {
	return (byte4[3] << 24) | (byte4[2] << 16) | (byte4[1] << 8) | byte4[0];
}
inline void Uint32ToByte4(uint32_t x, uint8_t *byte4) {
	byte4[0] = x & 0xffu;
	x >>= 8;
	byte4[1] = x & 0xffu;
	x >>= 8;
	byte4[2] = x & 0xffu;
	x >>= 8;
	byte4[3] = x;
}
inline float Byte4ToFloat(const uint8_t *byte4) {
	union {
		uint32_t in;
		float out;
	} u;
	u.in = Byte4ToUint32(byte4);
	return u.out;
}
inline void FloatToByte4(float x, uint8_t *byte4) {
	union {
		float in;
		uint32_t out;
	} u;
	u.in = x;
	Uint32ToByte4(u.out, byte4);
}

#endif // MATH_HPP
