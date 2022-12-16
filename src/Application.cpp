#include "Application.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <numeric>

#include <GLFW/glfw3.h>
#include <vulkan/vulkan_core.h>

#include "Resources.h"
#include "VulkanBackend/Buffer.h"

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
  globalPool_ = DescriptorPool::Builder(device_)
                   .setMaxSets(SwapChain::MAX_FRAMES_IN_FLIGHT)
                   .addPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, SwapChain::MAX_FRAMES_IN_FLIGHT)
                   .build();
}

Application::~Application() {
  model_->destroy(device_());
  vkDestroyDescriptorSetLayout(device_(), modelDescriptorSetLayout_, nullptr);
  vkDestroyDescriptorPool(device_(), modelDescriptorPool_, nullptr);
}

void Application::setupNodeDescriptorSet(Node *node) {
  if (node->mesh) {
    VkDescriptorSetAllocateInfo descriptorSetAllocInfo{};
    descriptorSetAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    descriptorSetAllocInfo.descriptorPool = modelDescriptorPool_;
    descriptorSetAllocInfo.pSetLayouts = &modelDescriptorSetLayout_;
    descriptorSetAllocInfo.descriptorSetCount = 1;
    (vkAllocateDescriptorSets(device_(), &descriptorSetAllocInfo, &node->mesh->uniformBuffer.descriptorSet));

    VkWriteDescriptorSet writeDescriptorSet{};
    writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writeDescriptorSet.descriptorCount = 1;
    writeDescriptorSet.dstSet = node->mesh->uniformBuffer.descriptorSet;
    writeDescriptorSet.dstBinding = 0;
    writeDescriptorSet.pBufferInfo = &node->mesh->uniformBuffer.descriptor;

    vkUpdateDescriptorSets(device_(), 1, &writeDescriptorSet, 0, nullptr);
  }
  for (auto &child : node->children) {
    setupNodeDescriptorSet(child);
  }
}

void Application::setupDescriptors() {
  /*
          Descriptor Pool
  */
  // HACK: Just set some amount of descriptors from this pool, this is a temp solution before
  // pipeline manager is implemented
  std::vector<VkDescriptorPoolSize> poolSizes = {
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000u * renderer_.getSwapChainImagecount()}};
  VkDescriptorPoolCreateInfo descriptorPoolCI{};
  descriptorPoolCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  descriptorPoolCI.poolSizeCount = 1;
  descriptorPoolCI.pPoolSizes = poolSizes.data();
  descriptorPoolCI.maxSets = 1000 * renderer_.getSwapChainImagecount();

  vkCreateDescriptorPool(device_(), &descriptorPoolCI, nullptr, &modelDescriptorPool_);

  /*
          Descriptor set layout
  */
  // Model node (matrices)
  {
    std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
        {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT, nullptr},
    };
    VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCI{};
    descriptorSetLayoutCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    descriptorSetLayoutCI.pBindings = setLayoutBindings.data();
    descriptorSetLayoutCI.bindingCount = static_cast<uint32_t>(setLayoutBindings.size());
    vkCreateDescriptorSetLayout(device_(), &descriptorSetLayoutCI, nullptr, &modelDescriptorSetLayout_);
  }
}

void Application::run() {
  glfwSetCursorPosCallback(window_.getWindow(), mouseCallback);
  glfwSetInputMode(window_.getWindow(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  setupDescriptors();
  auto globalSetLayout = DescriptorSetLayout::Builder(device_)
                             .addBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_VERTEX_BIT)
                             .build();

  std::vector<std::unique_ptr<Buffer>> uboBuffers(SwapChain::MAX_FRAMES_IN_FLIGHT);
  for (size_t i = 0; i < uboBuffers.size(); i++) {
    uboBuffers[i] = std::make_unique<Buffer>(device_, sizeof(Ubo), 1, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                             VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    uboBuffers[i]->map();
  }

  loadModel();

  // HACK: create temp pipeline
  VkPushConstantRange pushConstantRange{};
  pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
  pushConstantRange.offset = 0;
  pushConstantRange.size = sizeof(glm::mat4);
  std::vector<VkDescriptorSetLayout> descriptorSetLayouts{globalSetLayout->getDescriptorSetLayout(), modelDescriptorSetLayout_};
  VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
  pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  pipelineLayoutInfo.setLayoutCount = static_cast<uint32_t>(descriptorSetLayouts.size());
  pipelineLayoutInfo.pSetLayouts = descriptorSetLayouts.data();
  pipelineLayoutInfo.pushConstantRangeCount = 1;
  pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;
  if (vkCreatePipelineLayout(device_(), &pipelineLayoutInfo, nullptr, &tempPipelineLayout_) != VK_SUCCESS) {
    throw std::runtime_error("failed to create pipeline layout!");
  }

  ResourceManager::loadResourceFromFile<ShaderResource>("data/shaders/shader.frag.spv");
  ResourceManager::loadResourceFromFile<ShaderResource>("data/shaders/shader.vert.spv");

  model_->setMaterials(tempPipelineLayout_, renderer_.swapChainRenderPass());

  std::vector<VkDescriptorSet> globalDescriptorSets(SwapChain::MAX_FRAMES_IN_FLIGHT);
  for (size_t i = 0; i < globalDescriptorSets.size(); ++i) {
    auto bufferInfo = uboBuffers[i]->descriptorInfo();
    DescriptorWriter(*globalSetLayout, *globalPool_).writeBuffer(0, &bufferInfo).build(globalDescriptorSets[i]);
  }

  // Per-Node descriptor set
  for (auto &node : model_->nodes) {
    setupNodeDescriptorSet(node);
  }

  using namespace std::chrono;
  auto currentTime = high_resolution_clock::now();
  while (!window_.close()) {
    auto newTime = high_resolution_clock::now();
    float frameTime = duration<float, seconds::period>(newTime - currentTime).count();
    currentTime = newTime;

    camera.update(frameTime, window_.getWindow());
    float aspect = renderer_.getAspectRatio();
    camera.setPerspectiveProjection((60.f), aspect, 0.1f, 256.f);

    if (auto commandBuffer = renderer_.prepareFrame()) {
      int frameIndex = renderer_.getFrameIndex();
      FrameInfo frameInfo{frameIndex, frameTime, commandBuffer, camera, globalDescriptorSets[frameIndex]};

      Ubo ubo{};
      ubo.projection = camera.getProjection();
      ubo.view = camera.getViewMatrix();
      uboBuffers[frameIndex]->writeToBuffer(&ubo);
      uboBuffers[frameIndex]->flush();

      // render
      renderer_.beginRenderPass(commandBuffer);
      model_->draw(frameInfo, tempPipelineLayout_);
      renderer_.endRenderPass(commandBuffer);
      renderer_.submitFrame();
      model_->animator->updateAnimation(0, frameTime);
    }
    glfwPollEvents();
  }
  vkDeviceWaitIdle(device_());
}

void Application::loadModel() {
  std::string filename = "data/models/CesiumMan.gltf";
  model_ = std::make_unique<Model>(filename, &device_, device_.graphicsQueue);
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
