#include "Application.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <numeric>
#include <vulkan/vulkan_core.h>

#include "Buffer.h"
#include "Camera.h"
#include "Descriptors.h"
#include "GLFW/glfw3.h"
#include "RenderSystem.h"
#include "VulkanglTFModel.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

namespace {
bool firstMouse = true;
float lastX, lastY;
Camera camera{};
} // namespace

struct Ubo {
  glm::mat4 projection{};
  glm::mat4 view{};
  glm::vec4 lightPos{5.0f, 5.0f, -5.0f, 1.0f};
};

Application::Application() {
  loadModel();
  globalPool = DescriptorPool::Builder(device)
                   .setMaxSets(SwapChain::MAX_FRAMES_IN_FLIGHT)
                   .addPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, SwapChain::MAX_FRAMES_IN_FLIGHT)
                   .build();

  nodePool = DescriptorPool::Builder(device).addPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000).build();
}

Application::~Application() {}

void Application::createGlobalUBO() {}

void Application::setupDescriptors() {}

void Application::setupNodeDescriptorSet(vkglTF::Node *node) {
  if (node->mesh) {
  }
  for (auto &child : node->children) {
    setupNodeDescriptorSet(child);
  }
}

void Application::run() {
  glfwSetCursorPosCallback(window.getWindow(), mouseCallback);
  glfwSetInputMode(window.getWindow(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  std::vector<std::unique_ptr<Buffer>> uboBuffers(SwapChain::MAX_FRAMES_IN_FLIGHT);
  for (size_t i = 0; i < uboBuffers.size(); i++) {
    uboBuffers[i] = std::make_unique<Buffer>(device, sizeof(Ubo), 1, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                             VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    uboBuffers[i]->map();
  }

  auto globalSetLayout = DescriptorSetLayout::Builder(device)
                             .addBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_VERTEX_BIT)
                             .build();
  auto skinSetLayout = DescriptorSetLayout::Builder(device)
                           .addBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_VERTEX_BIT)
                           .build();

  std::vector<VkDescriptorSet> globalDescriptorSets(SwapChain::MAX_FRAMES_IN_FLIGHT);

  RenderSystem renderSystem{device, renderer.swapChainRenderPass(), globalSetLayout->getDescriptorSetLayout(),
                                  skinSetLayout->getDescriptorSetLayout()};

  for (size_t i = 0; i < globalDescriptorSets.size(); ++i) {
    auto bufferInfo = uboBuffers[i]->descriptorInfo();
    DescriptorWriter(*globalSetLayout, *globalPool).writeBuffer(0, &bufferInfo).build(globalDescriptorSets[i]);
  }
  for (auto &node : model->nodes) {
    if (node->mesh) {
      auto bufferInfo = node->mesh->uniformBuffer.descriptor;
      DescriptorWriter(*skinSetLayout, *nodePool).writeBuffer(0, &bufferInfo).build(node->mesh->uniformBuffer.descriptorSet);
    }
  }

  using namespace std::chrono;
  auto currentTime = high_resolution_clock::now();
  while (!window.close()) {
    auto newTime = high_resolution_clock::now();
    float frameTime = duration<float, seconds::period>(newTime - currentTime).count();
    currentTime = newTime;

    camera.update(frameTime, window.getWindow());
    float aspect = renderer.getAspectRatio();
    camera.setPerspectiveProjection((60.f), aspect, 0.1f, 256.f);

    if (auto commandBuffer = renderer.prepareFrame()) {
      int frameIndex = renderer.getFrameIndex();
      FrameInfo frameInfo{frameIndex, frameTime, commandBuffer, camera, globalDescriptorSets[frameIndex]};

      Ubo ubo{};
      ubo.projection = camera.getProjection();
      ubo.view = camera.getViewMatrix();
      uboBuffers[frameIndex]->writeToBuffer(&ubo);
      uboBuffers[frameIndex]->flush();

      // render
      renderer.beginRenderPass(commandBuffer);
      renderSystem.renderModel(frameInfo, *model);
      renderer.endRenderPass(commandBuffer);
      renderer.submitFrame();
      // model->updateAnimation(0, frameTime);
    }
    glfwPollEvents();
  }

  vkDeviceWaitIdle(device());
}

void Application::loadModel() {
  std::string filename = "data/models/animbox.gltf";
  model = std::make_unique<vkglTF::Model>(filename, &device, device.graphicsQueue);
  // model->loadFromFile(filename, &device, device.graphicsQueue, 1.0f);
};
// void Application::loadModel() { model = Model::createglTFModel(device, "data/models/FlightHelmet/FlightHelmet.gltf"); }

void mouseCallback(GLFWwindow *window, double xposIn, double yposIn) {
  float xpos = static_cast<float>(xposIn);
  float ypos = static_cast<float>(yposIn);

  if (firstMouse) {
    lastX = xpos;
    lastY = ypos;
    firstMouse = false;
  }

  float xoffset = xpos - lastX;
  float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

  lastX = xpos;
  lastY = ypos;

  camera.processMouseMovement(xoffset, yoffset);
}
