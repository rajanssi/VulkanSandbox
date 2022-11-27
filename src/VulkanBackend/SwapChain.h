#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Device.h"

class SwapChain {
public:
  // Public fields
  VkFormat swapChainImageFormat;
  VkRenderPass renderPass;
  VkExtent2D swapChainExtent;

  static constexpr int MAX_FRAMES_IN_FLIGHT = 2;

  SwapChain(Device &device, VkExtent2D extent);
  SwapChain(Device &device, VkExtent2D extent, std::shared_ptr<SwapChain> previous);

  ~SwapChain();

  SwapChain(const SwapChain &) = delete;
  void operator=(const SwapChain &) = delete;

  VkFramebuffer getFrameBuffer(int index) { return swapChainFramebuffers[index]; }
  VkImageView getImageView(int index) { return swapChainImageViews[index]; }

  float extentAspectRatio() { return static_cast<float>(swapChainExtent.width) / static_cast<float>(swapChainExtent.height); }
  size_t imageCount() { return swapChainImages.size(); }
  uint32_t width() { return swapChainExtent.width; }
  uint32_t height() { return swapChainExtent.height; }

  VkFormat findDepthFormat();
  VkResult acquireNextImage(uint32_t *imageIndex);
  VkResult submitCommandBuffers(const VkCommandBuffer *buffers, uint32_t *imageIndex);

private:
  VkSwapchainKHR swapChain;
  std::shared_ptr<SwapChain> oldSwapChain;

  VkExtent2D windowExtent;
  Device &device;
  std::vector<VkFramebuffer> swapChainFramebuffers;

  std::vector<VkImage> depthImages;
  std::vector<VkDeviceMemory> depthImageMemorys;
  std::vector<VkImageView> depthImageViews;
  std::vector<VkImage> swapChainImages;
  std::vector<VkImageView> swapChainImageViews;

  // Synchronization
  std::vector<VkSemaphore> imageAvailableSemaphores;
  std::vector<VkSemaphore> renderFinishedSemaphores;
  std::vector<VkFence> inFlightFences;
  std::vector<VkFence> imagesInFlight;
  size_t currentFrame = 0;

  void init();
  void createSwapChain();
  void createImageViews();
  void createDepthResources();
  void createRenderPass();
  void createFramebuffers();
  void createSyncObjects();

  // Helper functions
  VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &availableFormats);
  VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR> &availablePresentModes);
  VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities);
};
