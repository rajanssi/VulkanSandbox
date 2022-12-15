#pragma once

#include <string>
#include <vector>

#include <vulkan/vulkan_core.h>

#include "VulkanBackend/Device.h"

class ResourceManager;

class Resource {
public:
  Resource() = default;
  virtual ~Resource() = default;
  virtual void loadResource(const std::string &path) = 0;

private:
};

class ShaderResource : public Resource {
public:
  virtual void loadResource(const std::string &path) override;
  const std::vector<char> &getData();
private:
  std::vector<char> code_;

  std::vector<char> readFile(const std::string &path);
  void createShaderModule(Device& device, const std::vector<char> &code, VkShaderModule *shaderModule);
};

