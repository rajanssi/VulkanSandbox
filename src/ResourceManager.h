#pragma once

#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "Resources.h"
#include "VulkanBackend/Device.h"

class ResourceManager {
public:
  ResourceManager() = delete;

  void loadModelFromGLTF();

  template <typename T> static void loadResourceFromFile(const std::string &path) {
    auto s{std::make_shared<T>()};
    s->loadResource(path);
    resourceMap_[path] = s;
  }

  template <typename T> static T getResource(const std::string &key) {
    auto it = resourceMap_.find(key);
    if (it == resourceMap_.end()) {
      std::cout << key << '\n';
      throw std::runtime_error("couldn't find specified resource");
      return nullptr;
    }
    return dynamic_cast<T>(it->second.get());
  }

private:
  static std::unordered_map<std::string, std::shared_ptr<Resource>> resourceMap_;
};
