#pragma once

#include "VulkanBackend/Device.h"
#include "Primitive.h"
#include <glm/glm.hpp>

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
    glm::mat4 jointMatrix[128]{};
    float jointcount{0};
  } uniformBlock;
  Mesh(Device *device, glm::mat4 matrix);
  ~Mesh();
};
