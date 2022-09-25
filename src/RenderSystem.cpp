#include "RenderSystem.h"

#include <array>
#include <cassert>
#include <stdexcept>

#include "FrameInfo.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

/*
struct PushConstantData {
  glm::mat4 modelMatrix{1.f};
  glm::mat4 normalMatrix{1.f};
};
*/

RenderSystem::RenderSystem(Device &device, VkRenderPass renderPass, VkDescriptorSetLayout globalSetLayout,
                             VkDescriptorSetLayout imageSetLayout)
    : device{device} {
  createPipelineLayout(globalSetLayout, imageSetLayout);
  createPipeline(renderPass);
}

RenderSystem::RenderSystem(Device &device, VkRenderPass renderPass, VkDescriptorSetLayout globalSetLayout,
                                  VkDescriptorSetLayout imageSetLayout, VkDescriptorSetLayout skinSetLayout)
    : device{device} {
  createPipelineLayout(globalSetLayout, imageSetLayout, skinSetLayout);
  createPipeline(renderPass);
}

RenderSystem::~RenderSystem() { vkDestroyPipelineLayout(device(), pipelineLayout, nullptr); }

void RenderSystem::createPipelineLayout(VkDescriptorSetLayout globalSetLayout, VkDescriptorSetLayout imageSetLayout,
                                         VkDescriptorSetLayout skinSetLayout) {
  VkPushConstantRange pushConstantRange{};
  pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
  pushConstantRange.offset = 0;
  pushConstantRange.size = sizeof(glm::mat4);

  std::vector<VkDescriptorSetLayout> descriptorSetLayouts{globalSetLayout, skinSetLayout, imageSetLayout};

  VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
  pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  pipelineLayoutInfo.setLayoutCount = static_cast<uint32_t>(descriptorSetLayouts.size());
  pipelineLayoutInfo.pSetLayouts = descriptorSetLayouts.data();
  pipelineLayoutInfo.pushConstantRangeCount = 1;
  pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;
  if (vkCreatePipelineLayout(device(), &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) {
    throw std::runtime_error("failed to create pipeline layout!");
  }
}

void RenderSystem::createPipelineLayout(VkDescriptorSetLayout globalSetLayout, VkDescriptorSetLayout imageSetLayout) {
  VkPushConstantRange pushConstantRange{};
  pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
  pushConstantRange.offset = 0;
  pushConstantRange.size = sizeof(glm::mat4);

  std::vector<VkDescriptorSetLayout> descriptorSetLayouts{globalSetLayout, imageSetLayout};

  VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
  pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  pipelineLayoutInfo.setLayoutCount = static_cast<uint32_t>(descriptorSetLayouts.size());
  pipelineLayoutInfo.pSetLayouts = descriptorSetLayouts.data();
  pipelineLayoutInfo.pushConstantRangeCount = 1;
  pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;
  if (vkCreatePipelineLayout(device(), &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) {
    throw std::runtime_error("failed to create pipeline layout!");
  }
}

void RenderSystem::createPipeline(VkRenderPass renderPass) {
  assert(pipelineLayout != nullptr && "Cannot create pipeline before pipeline layout");

  PipelineConfigInfo pipelineConfig{};
  Pipeline::defaultPipelineConfigInfo(pipelineConfig);
  pipelineConfig.renderPass = renderPass;
  pipelineConfig.pipelineLayout = pipelineLayout;
  pipeline =
      std::make_unique<Pipeline>(device, "data/shaders/shader.vert.spv", "data/shaders/shader.frag.spv", pipelineConfig);
}

void RenderSystem::renderModel(FrameInfo &frameInfo, Model &model) {
  pipeline->bind(frameInfo.commandBuffer);

  vkCmdBindDescriptorSets(frameInfo.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1,
                          &frameInfo.globalDescriptorSet, 0, nullptr);

  model.draw(frameInfo.commandBuffer, pipelineLayout);
}
