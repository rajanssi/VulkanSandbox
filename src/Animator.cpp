#include "Animator.h"

#include <iostream>

Animator::~Animator() {
  animations.resize(0);
  for (auto skin : skins) {
    delete skin;
  }
  skins.resize(0);
}

void Animator::loadAnimations(tinygltf::Model &gltfModel) {
  for (tinygltf::Animation &anim : gltfModel.animations) {
    Animation animation{};
    animation.name = anim.name;
    if (anim.name.empty()) {
      animation.name = std::to_string(animations.size());
    }

    // Samplers
    for (auto &samp : anim.samplers) {
      AnimationSampler sampler{};

      if (samp.interpolation == "LINEAR") {
        sampler.interpolation = AnimationSampler::InterpolationType::LINEAR;
      }
      if (samp.interpolation == "STEP") {
        sampler.interpolation = AnimationSampler::InterpolationType::STEP;
      }
      if (samp.interpolation == "CUBICSPLINE") {
        sampler.interpolation = AnimationSampler::InterpolationType::CUBICSPLINE;
      }

      // Read sampler input time values
      {
        const tinygltf::Accessor &accessor = gltfModel.accessors[samp.input];
        const tinygltf::BufferView &bufferView = gltfModel.bufferViews[accessor.bufferView];
        const tinygltf::Buffer &buffer = gltfModel.buffers[bufferView.buffer];

        assert(accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT);

        const void *dataPtr = &buffer.data[accessor.byteOffset + bufferView.byteOffset];
        const float *buf = static_cast<const float *>(dataPtr);
        for (size_t index = 0; index < accessor.count; index++) {
          sampler.inputs.push_back(buf[index]);
        }

        for (auto input : sampler.inputs) {
          if (input < animation.start) {
            animation.start = input;
          };
          if (input > animation.end) {
            animation.end = input;
          }
        }
      }

      // Read sampler output T/R/S values
      {
        const tinygltf::Accessor &accessor = gltfModel.accessors[samp.output];
        const tinygltf::BufferView &bufferView = gltfModel.bufferViews[accessor.bufferView];
        const tinygltf::Buffer &buffer = gltfModel.buffers[bufferView.buffer];

        assert(accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT);

        const void *dataPtr = &buffer.data[accessor.byteOffset + bufferView.byteOffset];

        switch (accessor.type) {
        case TINYGLTF_TYPE_VEC3: {
          const glm::vec3 *buf = static_cast<const glm::vec3 *>(dataPtr);
          for (size_t index = 0; index < accessor.count; index++) {
            sampler.outputsVec4.push_back(glm::vec4(buf[index], 0.0f));
          }
          break;
        }
        case TINYGLTF_TYPE_VEC4: {
          const glm::vec4 *buf = static_cast<const glm::vec4 *>(dataPtr);
          for (size_t index = 0; index < accessor.count; index++) {
            sampler.outputsVec4.push_back(buf[index]);
          }
          break;
        }
        default: {
          std::cout << "unknown type" << std::endl;
          break;
        }
        }
      }

      animation.samplers.push_back(sampler);
    }

    // Channels
    for (auto &source : anim.channels) {
      AnimationChannel channel{};

      if (source.target_path == "rotation") {
        channel.path = AnimationChannel::PathType::ROTATION;
      }
      if (source.target_path == "translation") {
        channel.path = AnimationChannel::PathType::TRANSLATION;
      }
      if (source.target_path == "scale") {
        channel.path = AnimationChannel::PathType::SCALE;
      }
      if (source.target_path == "weights") {
        std::cout << "weights not yet supported, skipping channel" << std::endl;
        continue;
      }
      channel.samplerIndex = source.sampler;
      channel.node = Node::nodeFromIndex(source.target_node, nodes);
      if (!channel.node) {
        continue;
      }

      animation.channels.push_back(channel);
    }

    animations.push_back(animation);
  }
}

void Animator::updateAnimation(uint32_t index, float time) {
  if (animations.empty()) {
    std::cout << ".glTF does not contain animation." << std::endl;
    return;
  }
  if (index > static_cast<uint32_t>(animations.size()) - 1) {
    std::cout << "No animation with index " << index << std::endl;
    return;
  }
  Animation &animation = animations[index];

  bool updated = false;
  animationTimer += time;
  if (animationTimer > animations[index].end) {
    animationTimer -= animations[index].end;
  }

  for (auto &channel : animation.channels) {
    AnimationSampler &sampler = animation.samplers[channel.samplerIndex];
    if (sampler.inputs.size() > sampler.outputsVec4.size()) {
      continue;
    }

    for (size_t i = 0; i < sampler.inputs.size() - 1; i++) {
      if ((animationTimer >= sampler.inputs[i]) && (animationTimer <= sampler.inputs[i + 1])) {
        float u = std::max(0.0f, time - sampler.inputs[i]) / (sampler.inputs[i + 1] - sampler.inputs[i]);
        if (u <= 1.0f) {
          switch (channel.path) {
          case AnimationChannel::PathType::TRANSLATION: {
            glm::vec4 trans = glm::mix(sampler.outputsVec4[i], sampler.outputsVec4[i + 1], u);
            channel.node->translation = glm::vec3(trans);
            break;
          }
          case AnimationChannel::PathType::SCALE: {
            glm::vec4 trans = glm::mix(sampler.outputsVec4[i], sampler.outputsVec4[i + 1], u);
            channel.node->scale = glm::vec3(trans);
            break;
          }
          case AnimationChannel::PathType::ROTATION: {
            glm::quat q1;
            q1.x = sampler.outputsVec4[i].x;
            q1.y = sampler.outputsVec4[i].y;
            q1.z = sampler.outputsVec4[i].z;
            q1.w = sampler.outputsVec4[i].w;
            glm::quat q2;
            q2.x = sampler.outputsVec4[i + 1].x;
            q2.y = sampler.outputsVec4[i + 1].y;
            q2.z = sampler.outputsVec4[i + 1].z;
            q2.w = sampler.outputsVec4[i + 1].w;
            channel.node->rotation = glm::normalize(glm::slerp(q1, q2, u));
            break;
          }
          }
          updated = true;
        }
      }
    }
  }
  if (updated) {
    for (auto &node : nodes) {
      node->update();
    }
  }
}

void Animator::loadSkins(tinygltf::Model &gltfModel) {
  for (tinygltf::Skin &source : gltfModel.skins) {
    Skin *newSkin = new Skin{};
    newSkin->name = source.name;

    // Find skeleton root node
    if (source.skeleton > -1) {
      newSkin->skeletonRoot = Node::nodeFromIndex(source.skeleton, nodes);
    }

    // Find joint nodes
    for (int jointIndex : source.joints) {
      Node *node = Node::nodeFromIndex(jointIndex, nodes);
      if (node) {
        newSkin->joints.push_back(Node::nodeFromIndex(jointIndex, nodes));
      }
    }

    // Get inverse bind matrices from buffer
    if (source.inverseBindMatrices > -1) {
      const tinygltf::Accessor &accessor = gltfModel.accessors[source.inverseBindMatrices];
      const tinygltf::BufferView &bufferView = gltfModel.bufferViews[accessor.bufferView];
      const tinygltf::Buffer &buffer = gltfModel.buffers[bufferView.buffer];
      newSkin->inverseBindMatrices.resize(accessor.count);
      memcpy(newSkin->inverseBindMatrices.data(), &buffer.data[accessor.byteOffset + bufferView.byteOffset],
             accessor.count * sizeof(glm::mat4));
    }

    skins.push_back(newSkin);
  }
}

