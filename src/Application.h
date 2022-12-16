#pragma once

#include "Camera.h"
#include "Model.h"
#include "ResourceManager.h"
#include "Window.h"
#include "VulkanBackend/Descriptors.h"
#include "VulkanBackend/Device.h"
#include "VulkanBackend/Renderer.h"
#include "VulkanBackend/VulkanBuffer.h"

class Application {
public:
  // NOTE: default values for window size
  static constexpr int WIDHT = 800;
  static constexpr int HEIGHT = 600;

  Application();
  ~Application();
  Application(const Application &) = delete;
  Application &operator=(const Application &) = delete;

  void run();
private:
  // NOTE: Order is important, since it determines destruction order (think stack)
  Window window_{WIDHT, HEIGHT, "Vulkan Title"};
  Device device_{window_};
  Renderer renderer_{window_, device_};

  // HACK: all of these should be in appropriate locations
  struct UBOMatrices {
    glm::mat4 projection;
    glm::mat4 model;
    glm::mat4 view;
    glm::vec3 camPos;
  } shaderValues_;

  std::unique_ptr<DescriptorPool> globalPool_;
  std::unique_ptr<DescriptorPool> nodePool_;
  std::unique_ptr<Model> model_;

  VkDescriptorPool modelDescriptorPool_;
  VkDescriptorSetLayout modelDescriptorSetLayout_;
  VkPipelineLayout tempPipelineLayout_;

  void loadModel();
  void setupDescriptors();
  void setupNodeDescriptorSet(Node *node);
};

// HACK: Move this to a more appropriate place
void mouseCallback(GLFWwindow *window, double xposIn, double yposIn);
