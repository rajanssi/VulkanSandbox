#pragma once

#include "Mesh.h"
#include "Skin.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <string>
#include <vector>

struct Node {
  Node *parent;
  uint32_t index;
  std::vector<Node *> children;
  glm::mat4 matrix;
  std::string name;
  Mesh *mesh;
  Skin *skin;
  int32_t skinIndex = -1;
  glm::vec3 translation{};
  glm::vec3 scale{1.0f};
  glm::quat rotation{};
  glm::mat4 localMatrix();
  glm::mat4 getMatrix();
  void update();
  ~Node();

  static Node *findNode(Node* parent, uint32_t index);
  static Node *nodeFromIndex(uint32_t index, std::vector<Node *> &nodes);
};
