#include "Material.h"

#include <stdexcept>
#include <vulkan/vulkan_core.h>

#include "VulkanBackend/Pipeline.h"
#include "Model.h"

Material::Material(Device* device, VkPipelineLayout pipelineLayout, VkRenderPass renderPass) {
  PipelineConfigInfo pipelineConfig{};
  Pipeline::defaultPipelineConfigInfo(pipelineConfig);
  pipelineConfig.renderPass = renderPass;
  pipelineConfig.pipelineLayout = pipelineLayout;
  pipeline_ = std::make_unique<Pipeline>(*device, pipelineConfig);
}
