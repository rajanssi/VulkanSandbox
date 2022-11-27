#pragma once

#include <cassert>
#include <memory>
#include <vector>

#include "Device.h"
#include "SwapChain.h"
#include "Window.h"

class Renderer {
public:
  Renderer(Window &window, Device &device);
  ~Renderer();
  Renderer(const Renderer &) = delete;
  Renderer &operator=(const Renderer &) = delete;

  VkCommandBuffer prepareFrame();
  void submitFrame();
  void beginRenderPass(VkCommandBuffer commandBuffer);
  void endRenderPass(VkCommandBuffer commandBuffer);

  VkCommandBuffer currentCommandBuffer() const {
    assert(frameStarted && "cannot get cmdBuffer when frame not in progress");
    return commandBuffers[currentFrameIndex];
  };
  int getFrameIndex() const {
    assert(frameStarted && "Cannot get frame index if frame is not started");
    return currentFrameIndex;
  }

  int getSwapChainImagecount() const { return swapChain->imageCount(); };

  float getAspectRatio() const { return swapChain->extentAspectRatio(); };
  VkRenderPass swapChainRenderPass() const { return swapChain->renderPass; };

private:
  Window &window;
  Device &device;
  std::unique_ptr<SwapChain> swapChain;
  std::vector<VkCommandBuffer> commandBuffers;

  uint32_t currentImageIndex;
  int currentFrameIndex = 0;
  bool frameStarted = false;

  void recreateSwapChain();
};
