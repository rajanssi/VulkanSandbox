#pragma once

#include <vector>
#include <string>

#include <tiny_gltf.h>
#include <glm/glm.hpp>

struct Node;

struct Skin {
  std::string name;
  Node *skeletonRoot = nullptr;
  std::vector<glm::mat4> inverseBindMatrices;
  std::vector<Node *> joints;
};

