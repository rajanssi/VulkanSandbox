/*
 * Vulkan texture loader
 *
 * Copyright(C) by Sascha Willems - www.saschawillems.de
 *
 * This code is licensed under the MIT license(MIT) (http://opensource.org/licenses/MIT)
 */

#pragma once
#include "Device.h"

#include <fstream>
#include <stdlib.h>
#include <string>
#include <vector>

#include "vulkan/vulkan.h"

class Texture {
public:
  Device *device;
  VkImage image;
  VkImageLayout imageLayout;
  VkDeviceMemory deviceMemory;
  VkImageView view;
  uint32_t width, height;
  uint32_t mipLevels;
  uint32_t layerCount;
  VkDescriptorImageInfo descriptor;
  VkSampler sampler;

  void updateDescriptor();
  void destroy();
};

class Texture2D : public Texture {
public:
  void fromBuffer(void *buffer, VkDeviceSize bufferSize, VkFormat format, uint32_t texWidth, uint32_t texHeight,
                  Device *device, VkFilter filter = VK_FILTER_LINEAR,
                  VkImageUsageFlags imageUsageFlags = VK_IMAGE_USAGE_SAMPLED_BIT,
                  VkImageLayout imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  void setImageLayout(VkCommandBuffer cmdbuffer, VkImage image, VkImageLayout oldImageLayout, VkImageLayout newImageLayout,
                      VkImageSubresourceRange subresourceRange,
                      VkPipelineStageFlags srcStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                      VkPipelineStageFlags dstStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT);
};
