#version 460

// layout (set = 2, binding = 0) uniform sampler2D samplerColorMap;

layout (location = 0) in vec3 inNormal;
layout (location = 1) in vec3 inColor;
layout (location = 2) in vec3 inViewVec;
layout (location = 3) in vec3 inLightVec;

layout (location = 0) out vec4 outFragColor;

layout(push_constant) uniform Push {
  mat4 modelMatrix;
} node_matrix;

void main() {
  vec4 color = vec4(inColor, 1.0);
  vec3 N = normalize(inNormal);
  vec3 L = normalize(inLightVec);
  vec3 V = normalize(inViewVec);
  vec3 R = reflect(-L, N);
  vec3 diffuse = max(dot(N, L), 0.15) * inColor;
  vec3 specular = pow(max(dot(R, V), 0.0), 16.0) * vec3(0.75);
  outFragColor = vec4(color);
}

