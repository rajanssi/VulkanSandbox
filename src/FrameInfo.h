#pragma once

#include <vulkan/vulkan_core.h>

#include "Camera.h"

struct FrameInfo {
  int frameIndex;
  float frameTime;
  VkCommandBuffer commandBuffer;
  Camera &camera;
  VkDescriptorSet globalDescriptorSet;
};
