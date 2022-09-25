#pragma once

#include "Camera.h"
#include "Descriptors.h"
#include "Device.h"
#include "Model.h"
#include "Renderer.h"
#include "VulkanBuffer.h"
#include "Model.h"
#include "Window.h"

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
  Window window{WIDHT, HEIGHT, "Vulkan Title"};
  Device device{window};
  Renderer renderer{window, device};

  struct UBOMatrices {
    glm::mat4 projection;
    glm::mat4 model;
    glm::mat4 view;
    glm::vec3 camPos;
  } shaderValues;

  std::unique_ptr<DescriptorPool> globalPool;
  std::unique_ptr<DescriptorPool> nodePool;

  // std::unique_ptr<Model> model;
  std::unique_ptr<Model> model;

  VkDescriptorPool modelDescriptorPool;
  VkDescriptorSetLayout modelDescriptorSetLayout;

  void loadModel();
  void createGlobalUBO();
  void setupDescriptors();
  void setupNodeDescriptorSet(Node *node);
  void updateUniformBuffers();
};

// TODO: Move this to a more appropriate place
void mouseCallback(GLFWwindow *window, double xposIn, double yposIn);
