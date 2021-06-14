#version 450

#define ACCELERATED_SCENE_SET 0
#include "accelerated_scene.glsl"

layout(location = 0) out vec4 oColor;

layout(push_constant) uniform uuPushConstant { uint uWidth, uHeight, uViewType, uBeamEnable, uBeamSize; };
layout(set = 1, binding = 0) uniform uuCamera {
	mat4 uProjection;
	mat4 uInvProjection;
	mat4 uInvView;
};

vec3 GenRay() {
	vec2 coord = ivec2(gl_FragCoord.xy) / vec2(uWidth, uHeight);
	coord = coord * 2.0f - 1.0f;
	return normalize(mat3(uInvView) * (uInvProjection * vec4(coord, 1, 1)).xyz);
}

void main() {
	uint tri_idx;
	vec2 tri_uv;
	BVHIntersection(vec4(uInvView[3].xyz, 1e-6), GenRay(), tri_idx, tri_uv);

	if (tri_idx != 0xffffffffu) {
		oColor = vec4(TriangleFetchNormal(tri_idx, tri_uv) * 0.5 + 0.5, 1);
		// oColor = vec4(pow(TriangleFetchDiffuse(tri_idx, tri_uv), vec3(1.0
		// / 2.2)), 1);
	} else {
		oColor = vec4(0);
	}
}
