#include "Renderer.h"

#include <array>
#include <stdexcept>

Renderer::Renderer(Window &window, Device &device) : window{window}, device{device} {
  recreateSwapChain();
  // Create command buffers
  commandBuffers.resize(swapChain->MAX_FRAMES_IN_FLIGHT);

  VkCommandBufferAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocInfo.commandPool = device.commandPool;
  allocInfo.commandBufferCount = static_cast<uint32_t>(commandBuffers.size());

  if (vkAllocateCommandBuffers(device(), &allocInfo, commandBuffers.data()) != VK_SUCCESS) {
    throw std::runtime_error("failed to allocate command buffers!");
  }
}

Renderer::~Renderer() {
  // Free command buffers and clear them
  vkFreeCommandBuffers(device(), device.commandPool, static_cast<uint32_t>(commandBuffers.size()), commandBuffers.data());
  commandBuffers.clear();
}

void Renderer::recreateSwapChain() {
  auto extent = window.extent();
  while (extent.width == 0 || extent.height == 0) {
    extent = window.extent();
    glfwWaitEvents();
  }
  vkDeviceWaitIdle(device());

  if (swapChain == nullptr) {
    swapChain = std::make_unique<SwapChain>(device, extent);
  } else {
    swapChain = std::make_unique<SwapChain>(device, extent, std::move(swapChain));
  }
}

VkCommandBuffer Renderer::prepareFrame() {
  assert(!frameStarted && "Can't prepare a frame when frame is already in progress!");

  auto result = swapChain->acquireNextImage(&currentImageIndex);
  if (result == VK_ERROR_OUT_OF_DATE_KHR) {
    recreateSwapChain();
    return nullptr;
  }
  if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
    throw std::runtime_error("failed to acquire swap chain image!");
  }

  frameStarted = true;

  auto commandBuffer = currentCommandBuffer();

  VkCommandBufferBeginInfo beginInfo{};
  beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

  if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
    throw std::runtime_error("failed to begin recording command buffer!");
  }

  return commandBuffer;
}

void Renderer::submitFrame() {
  assert(frameStarted && "can't submit frame if frame not started");
  auto commandBuffer = currentCommandBuffer();

  if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
    throw std::runtime_error("failed to record command buffer!");
  }

  auto result = swapChain->submitCommandBuffers(&commandBuffer, &currentImageIndex);
  if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || window.resized) {
    window.resized = false;
    recreateSwapChain();
  } else if (result != VK_SUCCESS) {
    throw std::runtime_error("failed to present swap chain image!");
  }

  frameStarted = false;
  currentFrameIndex = (currentFrameIndex + 1) % swapChain->MAX_FRAMES_IN_FLIGHT;
}

void Renderer::beginRenderPass(VkCommandBuffer commandBuffer) {
  assert(frameStarted && "Can't begin render pass if frame is not in progress");
  assert(commandBuffer == currentCommandBuffer() && "can't begin render pass on a command buffer from different frame");

  VkRenderPassBeginInfo renderPassInfo{};
  renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
  renderPassInfo.renderPass = swapChain->renderPass;
  renderPassInfo.framebuffer = swapChain->getFrameBuffer(currentImageIndex);

  renderPassInfo.renderArea.offset = {0, 0};
  renderPassInfo.renderArea.extent = swapChain->swapChainExtent;

  std::array<VkClearValue, 2> clearValues{};
  clearValues[0].color = {0.8, 0.5, 0.6, 1.0f};
  clearValues[1].depthStencil = {1.0f, 0};
  renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
  renderPassInfo.pClearValues = clearValues.data();

  vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

  VkViewport viewport{};
  viewport.x = 0.0f;
  viewport.y = 0.0f;
  viewport.width = static_cast<float>(swapChain->swapChainExtent.width);
  viewport.height = static_cast<float>(swapChain->swapChainExtent.height);
  viewport.minDepth = 0.0f;
  viewport.maxDepth = 1.0f;
  VkRect2D scissor{{0, 0}, swapChain->swapChainExtent};
  vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
  vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
}

void Renderer::endRenderPass(VkCommandBuffer commandBuffer) {
  assert(frameStarted && "Can't end render pass if frame is not in progress");
  assert(commandBuffer == currentCommandBuffer() && "can't end render pass on a command buffer from different frame");
  vkCmdEndRenderPass(commandBuffer);
}
