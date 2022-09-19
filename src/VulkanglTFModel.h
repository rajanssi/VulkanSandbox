/**
 * Vulkan glTF model and texture loading class based on tinyglTF (https://github.com/syoyo/tinygltf)
 *
 * Copyright (C) 2018-2022 by Sascha Willems - www.saschawillems.de
 *
 * This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
 */

#pragma once

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

#include "Device.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include <tiny_gltf.h>


// Changing this value here also requires changing it in the vertex shader
#define MAX_NUM_JOINTS 128u

namespace vkglTF {
struct Node;

struct Primitive {
  uint32_t firstIndex;
  uint32_t indexCount;
  uint32_t vertexCount;
  bool hasIndices;
  Primitive(uint32_t firstIndex, uint32_t indexCount, uint32_t vertexCount);
};

struct Mesh {
  Device *device;
  std::vector<Primitive *> primitives;
  struct UniformBuffer {
    VkBuffer buffer;
    VkDeviceMemory memory;
    VkDescriptorBufferInfo descriptor;
    VkDescriptorSet descriptorSet;
    void *mapped;
  } uniformBuffer;
  struct UniformBlock {
    glm::mat4 matrix;
    glm::mat4 jointMatrix[MAX_NUM_JOINTS]{};
    float jointcount{0};
  } uniformBlock;
  Mesh(Device *device, glm::mat4 matrix);
  ~Mesh();
};

struct Skin {
  std::string name;
  Node *skeletonRoot = nullptr;
  std::vector<glm::mat4> inverseBindMatrices;
  std::vector<Node *> joints;
};

struct Node {
  Node *parent;
  uint32_t index;
  std::vector<Node *> children;
  glm::mat4 matrix;
  std::string name;
  Mesh *mesh;
  Skin *skin;
  int32_t skinIndex = -1;
  glm::vec3 translation{};
  glm::vec3 scale{1.0f};
  glm::quat rotation{};
  glm::mat4 localMatrix();
  glm::mat4 getMatrix();
  void update();
  ~Node();
};

struct AnimationChannel {
  enum PathType { TRANSLATION, ROTATION, SCALE };
  PathType path;
  Node *node;
  uint32_t samplerIndex;
};

struct AnimationSampler {
  enum InterpolationType { LINEAR, STEP, CUBICSPLINE };
  InterpolationType interpolation;
  std::vector<float> inputs;
  std::vector<glm::vec4> outputsVec4;
};

struct Animation {
  std::string name;
  std::vector<AnimationSampler> samplers;
  std::vector<AnimationChannel> channels;
  float start = std::numeric_limits<float>::max();
  float end = std::numeric_limits<float>::min();
};

class Model {
public:
  Device *device;

  struct Vertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec4 joint0;
    glm::vec4 weight0;
    glm::vec4 color;
  };

  struct Vertices {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory;
  } vertices;
  struct Indices {
    int count;
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory;
  } indices;

  glm::mat4 aabb;

  std::vector<Node *> nodes;
  std::vector<Node *> linearNodes;

  std::vector<Skin *> skins;

  std::vector<Animation> animations;
  std::vector<std::string> extensions;

  struct LoaderInfo {
    uint32_t *indexBuffer;
    Vertex *vertexBuffer;
    size_t indexPos = 0;
    size_t vertexPos = 0;
  };

  Model(std::string filename, Device *device, VkQueue transferQueue) { loadFromFile(filename, device, transferQueue, 1.0f); }

  void destroy(VkDevice device);
  void loadNode(vkglTF::Node *parent, const tinygltf::Node &node, uint32_t nodeIndex, const tinygltf::Model &model,
                LoaderInfo &loaderInfo, float globalscale);
  void getNodeProps(const tinygltf::Node &node, const tinygltf::Model &model, size_t &vertexCount, size_t &indexCount);
  void loadSkins(tinygltf::Model &gltfModel);
  void loadAnimations(tinygltf::Model &gltfModel);
  void loadFromFile(std::string filename, Device *device, VkQueue transferQueue, float scale = 1.0f);
  void drawNode(Node *node, VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);
  void draw(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);
  void updateAnimation(uint32_t index, float time);
  Node *findNode(Node *parent, uint32_t index);
  Node *nodeFromIndex(uint32_t index);
  static std::vector<VkVertexInputBindingDescription> getBindingDescriptions();
  static std::vector<VkVertexInputAttributeDescription> getAttributeDescriptions();
};
} // namespace vkglTF
