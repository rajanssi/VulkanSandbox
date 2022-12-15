#include "ResourceManager.h"

/*
void ResourceManager::loadResourceFromFile(ResourceType resType, std::string &path) {
  switch (resType) {
  case ResourceType::SHADER: {
    auto s{ std::make_shared<ShaderResource>()};
    s->loadResource(path);
    resourceMap[path] = s;
  }
  default:
    break;
  }
}

Resource* ResourceManager::getResource(std::string &key) {
  auto it = resourceMap.find(key);
  if (it == resourceMap.end()) {
    return nullptr;
  }
  return resourceMap[key].get();
}


template<typename T>
T ResourceManager::getResource(std::string &key) {

  auto it = resourceMap.find(key);
  if (it == resourceMap.end()) {
    return nullptr;
  }
  auto res = it->second.get();
  return dynamic_cast<T>(res);
}

*/
