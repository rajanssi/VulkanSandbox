#include "Resources.h"

#include <fstream>
#include <iostream>
#include <vector>

void MeshResource::loadResource(const std::string &path) {

}

std::vector<char> ShaderResource::readFile(const std::string &path) {
  std::ifstream file{path, std::ios::ate | std::ios::binary};

  if (!file.is_open()) {
    throw std::runtime_error("failed to open file: " + path);
  }

  size_t fileSize = static_cast<size_t>(file.tellg());
  std::vector<char> buffer(fileSize);

  file.seekg(0);
  file.read(buffer.data(), fileSize);

  file.close();
  return buffer;
}

void ShaderResource::loadResource(const std::string &path) { code_ = readFile(path); }

const std::vector<char> &ShaderResource::getData() { return code_; }

void TextureResource::loadResource(const std::string &path) {
  std::cout << "unimplemented!\n";

}
