#pragma once

#include "VulkanBackend/Device.h"
#include "Node.h"
#include "Skin.h"

#include <glm/glm.hpp>

#include <limits>
#include <vector>

class Animator {
private:
  struct AnimationChannel {
    enum PathType { TRANSLATION, ROTATION, SCALE };
    PathType path;
    Node *node;
    uint32_t samplerIndex;
  };

  struct AnimationSampler {
    enum InterpolationType { LINEAR, STEP, CUBICSPLINE };
    InterpolationType interpolation;
    std::vector<float> inputs;
    std::vector<glm::vec4> outputsVec4;
  };

  struct Animation {
    std::string name;
    std::vector<AnimationSampler> samplers;
    std::vector<AnimationChannel> channels;
    float start = std::numeric_limits<float>::max();
    float end = std::numeric_limits<float>::min();
  };

  std::vector<Animation> animations;
  std::vector<Node *> &nodes;

  float animationTimer = 0.0f;

public:
  std::vector<Skin *> skins;

  Animator(std::vector<Node *> &nodes) : nodes{nodes} {};
  ~Animator();

  void loadAnimations(tinygltf::Model &gltfModel);
  void updateAnimation(uint32_t index, float time);
  void loadSkins(tinygltf::Model &gltfModel);
};
