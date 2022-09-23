#version 460

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inColor;
layout (location = 3) in vec4 inJoint0;
layout (location = 4) in vec4 inWeight0;

layout (set = 0, binding = 0) uniform UBOScene {
	mat4 projection;
	mat4 view;
	vec4 lightPos;
} ubo;

layout(push_constant) uniform Push {
	mat4 model;
} primitive;

#define MAX_NUM_JOINTS 128

layout (set = 1, binding = 0) uniform UBONode {
	mat4 matrix;
	mat4 jointMatrix[MAX_NUM_JOINTS];
	float jointCount;
} node;

layout (location = 0) out vec3 outNormal;
layout (location = 1) out vec3 outColor;
layout (location = 2) out vec3 outViewVec;
layout (location = 3) out vec3 outLightVec;

void main()
{
  outColor = inColor;

	vec4 locPos;
	if (node.jointCount > 0.0) {
		// Mesh is skinned
		mat4 skinMat =
			inWeight0.x * node.jointMatrix[int(inJoint0.x)] +
			inWeight0.y * node.jointMatrix[int(inJoint0.y)] +
			inWeight0.z * node.jointMatrix[int(inJoint0.z)] +
			inWeight0.w * node.jointMatrix[int(inJoint0.w)];

		locPos = primitive.model * node.matrix * skinMat * vec4(inPos, 1.0);
		outNormal = normalize(transpose(inverse(mat3(primitive.model * node.matrix * skinMat))) * inNormal);
	} else {
		locPos = primitive.model * node.matrix * vec4(inPos, 1.0);
		outNormal = normalize(transpose(inverse(mat3(primitive.model * node.matrix))) * inNormal);
	}
	locPos.y = -locPos.y;

	vec3 outWorldPos = locPos.xyz / locPos.w;
	gl_Position =  ubo.projection * ubo.view * vec4(outWorldPos, 1.0);

  vec4 pos = ubo.view * vec4(inPos, 1.0);
	vec3 lPos = mat3(ubo.view) * ubo.lightPos.xyz;
	outLightVec = lPos - pos.xyz;
	outViewVec = -pos.xyz;
}
