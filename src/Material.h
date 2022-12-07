#pragma once

#include <memory>

#include "VulkanBackend/Device.h"
#include "VulkanBackend/Pipeline.h"

// TODO: for now just "encapsulates" pipeline
class Material {
public:
  Material(Device* device, VkPipelineLayout pipelineLayout, VkRenderPass renderPass);
  std::unique_ptr<Pipeline> pipeline_;
private:
  void createShaderModule(Device& device, const std::vector<char> &code, VkShaderModule *shaderModule);
};
