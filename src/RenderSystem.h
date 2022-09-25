#pragma once

#include <memory>
#include <vector>

#include "Camera.h"
#include "Device.h"
#include "FrameInfo.h"
#include "Model.h"
#include "Pipeline.h"
#include "Model.h"

class RenderSystem {
public:
  RenderSystem(Device &device, VkRenderPass renderPass, VkDescriptorSetLayout globalSetLayout,
               VkDescriptorSetLayout imageSetLayout);
  RenderSystem(Device &device, VkRenderPass renderPass, VkDescriptorSetLayout globalSetLayout,
               VkDescriptorSetLayout imageSetLayout, VkDescriptorSetLayout skinSetLayout);
  ~RenderSystem();
  RenderSystem(const RenderSystem &) = delete;
  RenderSystem &operator=(const RenderSystem &) = delete;

  void renderModel(FrameInfo &frameInfo, Model &model);

private:
  // NOTE:
  // Order is important, since it determines destruction order (basically stack)
  Device &device;
  std::unique_ptr<Pipeline> pipeline;
  VkPipelineLayout pipelineLayout;

  void createPipelineLayout(VkDescriptorSetLayout globalSetLayout, VkDescriptorSetLayout imageSetLayout);
  void createPipelineLayout(VkDescriptorSetLayout globalSetLayout, VkDescriptorSetLayout imageSetLayout,
                            VkDescriptorSetLayout skinsetLayout);
  void createPipeline(VkRenderPass renderPass);
};
