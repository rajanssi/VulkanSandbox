#pragma once

#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "VulkanBackend/Device.h"
#include "Resources.h"

class ResourceManager {
public:
  ResourceManager(Device &device) : device_{device} {};

  void loadModelFromGLTF();

  template<typename T>
  void loadResourceFromFile(const std::string &path) {
    auto s{ std::make_shared<ShaderResource>()};
    s->loadResource(path);
    resourceMap_[path] = s;
  }

  template<typename T>
  T getResource(const std::string &key) {
    auto it = resourceMap_.find(key);
    if (it == resourceMap_.end()) {
      std::cerr << key << '\n';
      throw std::runtime_error("couldn't find specified resource");
      // return nullptr;
    }
    return dynamic_cast<T>(it->second.get());
  }

private:
  Device &device_;

  std::unordered_map<std::string, std::shared_ptr<Resource>> resourceMap_;
};
