#include "Node.h"

#include <cstring>

// Node
glm::mat4 Node::localMatrix() {
  return glm::translate(glm::mat4(1.0f), translation) * glm::mat4(rotation) * glm::scale(glm::mat4(1.0f), scale) * matrix;
}

glm::mat4 Node::getMatrix() {
  glm::mat4 m = localMatrix();
  Node *p = parent;
  while (p) {
    m = p->localMatrix() * m;
    p = p->parent;
  }
  return m;
}

void Node::update() {
  if (mesh) {
    glm::mat4 m = getMatrix();
    if (skin) {
      mesh->uniformBlock.matrix = m;
      // Update join matrices
      glm::mat4 inverseTransform = glm::inverse(m);
      size_t numJoints = std::min((uint32_t)skin->joints.size(), 128u);
      for (size_t i = 0; i < numJoints; i++) {
        Node *jointNode = skin->joints[i];
        glm::mat4 jointMat = jointNode->getMatrix() * skin->inverseBindMatrices[i];
        jointMat = inverseTransform * jointMat;
        mesh->uniformBlock.jointMatrix[i] = jointMat;
      }
      mesh->uniformBlock.jointcount = (float)numJoints;
      memcpy(mesh->uniformBuffer.mapped, &mesh->uniformBlock, sizeof(mesh->uniformBlock));
    } else {
      memcpy(mesh->uniformBuffer.mapped, &m, sizeof(glm::mat4));
    }
  }

  for (auto &child : children) {
    child->update();
  }
}

Node::~Node() {
  if (mesh) {
    delete mesh;
  }
  for (auto &child : children) {
    delete child;
  }
}
Node *Node::findNode(Node *parent, uint32_t index) {
  Node *nodeFound = nullptr;
  if (parent->index == index) {
    return parent;
  }
  for (auto &child : parent->children) {
    nodeFound = Node::findNode(child, index);
    if (nodeFound) {
      break;
    }
  }
  return nodeFound;
}

Node *Node::nodeFromIndex(uint32_t index, std::vector<Node *> &nodes) {
  Node *nodeFound = nullptr;
  for (auto &node : nodes) {
    nodeFound = findNode(node, index);
    if (nodeFound) {
      break;
    }
  }
  return nodeFound;
}
