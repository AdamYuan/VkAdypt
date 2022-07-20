#version 450

#define ACCELERATED_SCENE_SET 0
#include "accelerated_scene.glsl"
#define CAMERA_SET 1
#include "camera.glsl"

layout(location = 0) out vec4 oColor;
layout(push_constant) uniform uuPushConstant { uint uWidth, uHeight, uViewType, uBeamEnable, uBeamSize; };

void main() {
	uint tri_idx;
	vec2 tri_uv;
	BVHIntersection(vec4(uPosition.xyz, 1e-6), CameraGenRay(gl_FragCoord.xy / vec2(uWidth, uHeight)), tri_idx, tri_uv);

	if (tri_idx != 0xffffffffu) {
		// oColor = vec4(TriangleFetchNormal(tri_idx, tri_uv) * 0.5 + 0.5, 1);
		oColor = vec4(pow(TriangleFetchDiffuse(tri_idx, tri_uv), vec3(1.0 / 2.2)), 1);
	} else {
		oColor = vec4(0);
	}
}
