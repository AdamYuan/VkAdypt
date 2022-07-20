#ifndef CAMERA_SET
#define CAMERA_SET 1
#endif

layout(set = CAMERA_SET, binding = 0) uniform uuCamera { vec4 uPosition, uLook, uSide, uUp; };

vec3 CameraGenRay(vec2 coord) {
	coord = coord * 2.0f - 1.0f;
	return normalize(uLook.xyz - uSide.xyz * coord.x - uUp.xyz * coord.y);
}
