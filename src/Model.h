#pragma once

#include "Animator.h"
#include "FrameInfo.h"
#include "VulkanBackend/Device.h"
#include "VulkanBackend/Pipeline.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <string>
#include <vector>
#include <vulkan/vulkan_core.h>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include <tiny_gltf.h>

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

  std::vector<Node *> nodes;
  std::vector<Node *> linearNodes;

  Material *material;

  std::unique_ptr<Animator> animator;

  struct LoaderInfo {
    uint32_t *indexBuffer;
    Vertex *vertexBuffer;
    size_t indexPos = 0;
    size_t vertexPos = 0;
  };

  Model(std::string filename, Device *device, VkQueue transferQueue) { loadFromFile(filename, device, transferQueue, 1.0f); }

  void destroy(VkDevice device);
  void loadNode(Node *parent, const tinygltf::Node &node, uint32_t nodeIndex, const tinygltf::Model &model,
                LoaderInfo &loaderInfo, float globalscale);
  void getNodeProps(const tinygltf::Node &node, const tinygltf::Model &model, size_t &vertexCount, size_t &indexCount);
  void loadFromFile(std::string filename, Device *device, VkQueue transferQueue, float scale = 1.0f);
  void setMaterials(VkPipelineLayout pipelineLayout, VkRenderPass renderPass);
  void drawNode(Node *node, VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout);
  void draw(FrameInfo &frameInfo, VkPipelineLayout pipelineLayout);
  static std::vector<VkVertexInputBindingDescription> getBindingDescriptions();
  static std::vector<VkVertexInputAttributeDescription> getAttributeDescriptions();
};
