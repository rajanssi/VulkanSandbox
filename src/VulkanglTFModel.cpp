/**
 * Vulkan glTF model and texture loading class based on tinyglTF (https://github.com/syoyo/tinygltf)
 *
 * Copyright (C) 2018-2022 by Sascha Willems - www.saschawillems.de
 *
 * This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
 */

#include "VulkanglTFModel.h"
#include <vulkan/vulkan_core.h>

namespace vkglTF {

std::vector<VkVertexInputBindingDescription> Model::getBindingDescriptions() {
  std::vector<VkVertexInputBindingDescription> bindingDescriptions(1);
  bindingDescriptions[0].binding = 0;
  bindingDescriptions[0].stride = sizeof(Vertex);
  bindingDescriptions[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
  return bindingDescriptions;
}

std::vector<VkVertexInputAttributeDescription> Model::getAttributeDescriptions() {
  std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};

  attributeDescriptions.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos)});
  attributeDescriptions.push_back({1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, normal)});
  attributeDescriptions.push_back({2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, color)});
  attributeDescriptions.push_back({3, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(Vertex, joint0)});
  attributeDescriptions.push_back({4, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(Vertex, weight0)});

  return attributeDescriptions;
}

// Primitive
Primitive::Primitive(uint32_t firstIndex, uint32_t indexCount, uint32_t vertexCount)
    : firstIndex(firstIndex), indexCount(indexCount), vertexCount(vertexCount) {
  hasIndices = indexCount > 0;
};

// Mesh
Mesh::Mesh(Device *device, glm::mat4 matrix) {
  this->device = device;
  this->uniformBlock.matrix = matrix;
  device->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, sizeof(uniformBlock),
                       &uniformBuffer.buffer, &uniformBuffer.memory, &uniformBlock);
  vkMapMemory(device->device, uniformBuffer.memory, 0, sizeof(uniformBlock), 0, &uniformBuffer.mapped);
  uniformBuffer.descriptor = {uniformBuffer.buffer, 0, sizeof(uniformBlock)};
};

Mesh::~Mesh() {
  vkDestroyBuffer(device->device, uniformBuffer.buffer, nullptr);
  vkFreeMemory(device->device, uniformBuffer.memory, nullptr);
  for (Primitive *p : primitives)
    delete p;
}

// Node
glm::mat4 Node::localMatrix() {
  return glm::translate(glm::mat4(1.0f), translation) * glm::mat4(rotation) * glm::scale(glm::mat4(1.0f), scale) * matrix;
}

glm::mat4 Node::getMatrix() {
  glm::mat4 m = localMatrix();
  vkglTF::Node *p = parent;
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
      size_t numJoints = std::min((uint32_t)skin->joints.size(), MAX_NUM_JOINTS);
      for (size_t i = 0; i < numJoints; i++) {
        vkglTF::Node *jointNode = skin->joints[i];
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

// Model

void Model::destroy(VkDevice device) {
  if (vertices.buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(device, vertices.buffer, nullptr);
    vkFreeMemory(device, vertices.memory, nullptr);
    vertices.buffer = VK_NULL_HANDLE;
  }
  if (indices.buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(device, indices.buffer, nullptr);
    vkFreeMemory(device, indices.memory, nullptr);
    indices.buffer = VK_NULL_HANDLE;
  }
  for (auto node : nodes) {
    delete node;
  }
  animations.resize(0);
  nodes.resize(0);
  linearNodes.resize(0);
  extensions.resize(0);
  for (auto skin : skins) {
    delete skin;
  }
  skins.resize(0);
};

void Model::loadNode(vkglTF::Node *parent, const tinygltf::Node &node, uint32_t nodeIndex, const tinygltf::Model &model,
                     LoaderInfo &loaderInfo, float globalscale) {
  vkglTF::Node *newNode = new Node{};
  newNode->index = nodeIndex;
  newNode->parent = parent;
  newNode->name = node.name;
  newNode->skinIndex = node.skin;
  newNode->matrix = glm::mat4(1.0f);

  // Generate local node matrix
  glm::vec3 translation = glm::vec3(0.0f);
  if (node.translation.size() == 3) {
    translation = glm::make_vec3(node.translation.data());
    newNode->translation = translation;
  }
  glm::mat4 rotation = glm::mat4(1.0f);
  if (node.rotation.size() == 4) {
    glm::quat q = glm::make_quat(node.rotation.data());
    newNode->rotation = glm::mat4(q);
  }
  glm::vec3 scale = glm::vec3(1.0f);
  if (node.scale.size() == 3) {
    scale = glm::make_vec3(node.scale.data());
    newNode->scale = scale;
  }
  if (node.matrix.size() == 16) {
    newNode->matrix = glm::make_mat4x4(node.matrix.data());
  };

  // Node with children
  if (node.children.size() > 0) {
    for (size_t i = 0; i < node.children.size(); i++) {
      loadNode(newNode, model.nodes[node.children[i]], node.children[i], model, loaderInfo, globalscale);
    }
  }

  // Node contains mesh data
  if (node.mesh > -1) {
    const tinygltf::Mesh mesh = model.meshes[node.mesh];
    Mesh *newMesh = new Mesh(device, newNode->matrix);
    for (size_t j = 0; j < mesh.primitives.size(); j++) {
      const tinygltf::Primitive &primitive = mesh.primitives[j];
      uint32_t vertexStart = loaderInfo.vertexPos;
      uint32_t indexStart = loaderInfo.indexPos;
      uint32_t indexCount = 0;
      uint32_t vertexCount = 0;
      glm::vec3 posMin{};
      glm::vec3 posMax{};
      bool hasSkin = false;
      bool hasIndices = primitive.indices > -1;
      // Vertices
      {
        const float *bufferPos = nullptr;
        const float *bufferNormals = nullptr;
        const float *bufferColorSet0 = nullptr;
        const void *bufferJoints = nullptr;
        const float *bufferWeights = nullptr;

        int posByteStride;
        int normByteStride;
        int uv0ByteStride;
        int uv1ByteStride;
        int color0ByteStride;
        int jointByteStride;
        int weightByteStride;

        int jointComponentType;

        // Position attribute is required
        assert(primitive.attributes.find("POSITION") != primitive.attributes.end());

        const tinygltf::Accessor &posAccessor = model.accessors[primitive.attributes.find("POSITION")->second];
        const tinygltf::BufferView &posView = model.bufferViews[posAccessor.bufferView];
        bufferPos = reinterpret_cast<const float *>(
            &(model.buffers[posView.buffer].data[posAccessor.byteOffset + posView.byteOffset]));
        posMin = glm::vec3(posAccessor.minValues[0], posAccessor.minValues[1], posAccessor.minValues[2]);
        posMax = glm::vec3(posAccessor.maxValues[0], posAccessor.maxValues[1], posAccessor.maxValues[2]);
        vertexCount = static_cast<uint32_t>(posAccessor.count);
        posByteStride = posAccessor.ByteStride(posView) ? (posAccessor.ByteStride(posView) / sizeof(float))
                                                        : tinygltf::GetNumComponentsInType(TINYGLTF_TYPE_VEC3);

        if (primitive.attributes.find("NORMAL") != primitive.attributes.end()) {
          const tinygltf::Accessor &normAccessor = model.accessors[primitive.attributes.find("NORMAL")->second];
          const tinygltf::BufferView &normView = model.bufferViews[normAccessor.bufferView];
          bufferNormals = reinterpret_cast<const float *>(
              &(model.buffers[normView.buffer].data[normAccessor.byteOffset + normView.byteOffset]));
          normByteStride = normAccessor.ByteStride(normView) ? (normAccessor.ByteStride(normView) / sizeof(float))
                                                             : tinygltf::GetNumComponentsInType(TINYGLTF_TYPE_VEC3);
        }

        // Vertex colors
        if (primitive.attributes.find("COLOR_0") != primitive.attributes.end()) {
          const tinygltf::Accessor &accessor = model.accessors[primitive.attributes.find("COLOR_0")->second];
          const tinygltf::BufferView &view = model.bufferViews[accessor.bufferView];
          bufferColorSet0 =
              reinterpret_cast<const float *>(&(model.buffers[view.buffer].data[accessor.byteOffset + view.byteOffset]));
          color0ByteStride = accessor.ByteStride(view) ? (accessor.ByteStride(view) / sizeof(float))
                                                       : tinygltf::GetNumComponentsInType(TINYGLTF_TYPE_VEC3);
        }

        // Skinning
        // Joints
        if (primitive.attributes.find("JOINTS_0") != primitive.attributes.end()) {
          const tinygltf::Accessor &jointAccessor = model.accessors[primitive.attributes.find("JOINTS_0")->second];
          const tinygltf::BufferView &jointView = model.bufferViews[jointAccessor.bufferView];
          bufferJoints = &(model.buffers[jointView.buffer].data[jointAccessor.byteOffset + jointView.byteOffset]);
          jointComponentType = jointAccessor.componentType;
          jointByteStride = jointAccessor.ByteStride(jointView)
                                ? (jointAccessor.ByteStride(jointView) / tinygltf::GetComponentSizeInBytes(jointComponentType))
                                : tinygltf::GetNumComponentsInType(TINYGLTF_TYPE_VEC4);
        }

        if (primitive.attributes.find("WEIGHTS_0") != primitive.attributes.end()) {
          const tinygltf::Accessor &weightAccessor = model.accessors[primitive.attributes.find("WEIGHTS_0")->second];
          const tinygltf::BufferView &weightView = model.bufferViews[weightAccessor.bufferView];
          bufferWeights = reinterpret_cast<const float *>(
              &(model.buffers[weightView.buffer].data[weightAccessor.byteOffset + weightView.byteOffset]));
          weightByteStride = weightAccessor.ByteStride(weightView) ? (weightAccessor.ByteStride(weightView) / sizeof(float))
                                                                   : tinygltf::GetNumComponentsInType(TINYGLTF_TYPE_VEC4);
        }

        hasSkin = (bufferJoints && bufferWeights);

        for (size_t v = 0; v < posAccessor.count; v++) {
          Vertex &vert = loaderInfo.vertexBuffer[loaderInfo.vertexPos];
          vert.pos = glm::vec4(glm::make_vec3(&bufferPos[v * posByteStride]), 1.0f);
          vert.normal =
              glm::normalize(glm::vec3(bufferNormals ? glm::make_vec3(&bufferNormals[v * normByteStride]) : glm::vec3(0.0f)));
          vert.color =
              bufferColorSet0 ? glm::make_vec4(&bufferColorSet0[v * color0ByteStride]) : glm::vec4(0.1f, 0.1f, 0.1f, 1.0f);

          if (hasSkin) {
            switch (jointComponentType) {
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
              const uint16_t *buf = static_cast<const uint16_t *>(bufferJoints);
              vert.joint0 = glm::vec4(glm::make_vec4(&buf[v * jointByteStride]));
              break;
            }
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
              const uint8_t *buf = static_cast<const uint8_t *>(bufferJoints);
              vert.joint0 = glm::vec4(glm::make_vec4(&buf[v * jointByteStride]));
              break;
            }
            default:
              // Not supported by spec
              std::cerr << "Joint component type " << jointComponentType << " not supported!" << std::endl;
              break;
            }
          } else {
            vert.joint0 = glm::vec4(0.0f);
          }
          vert.weight0 = hasSkin ? glm::make_vec4(&bufferWeights[v * weightByteStride]) : glm::vec4(0.0f);
          // Fix for all zero weights
          if (glm::length(vert.weight0) == 0.0f) {
            vert.weight0 = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
          }
          loaderInfo.vertexPos++;
        }
      }
      // Indices
      if (hasIndices) {
        const tinygltf::Accessor &accessor = model.accessors[primitive.indices > -1 ? primitive.indices : 0];
        const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
        const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

        indexCount = static_cast<uint32_t>(accessor.count);
        const void *dataPtr = &(buffer.data[accessor.byteOffset + bufferView.byteOffset]);

        switch (accessor.componentType) {
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_INT: {
          const uint32_t *buf = static_cast<const uint32_t *>(dataPtr);
          for (size_t index = 0; index < accessor.count; index++) {
            loaderInfo.indexBuffer[loaderInfo.indexPos] = buf[index] + vertexStart;
            loaderInfo.indexPos++;
          }
          break;
        }
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_SHORT: {
          const uint16_t *buf = static_cast<const uint16_t *>(dataPtr);
          for (size_t index = 0; index < accessor.count; index++) {
            loaderInfo.indexBuffer[loaderInfo.indexPos] = buf[index] + vertexStart;
            loaderInfo.indexPos++;
          }
          break;
        }
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_BYTE: {
          const uint8_t *buf = static_cast<const uint8_t *>(dataPtr);
          for (size_t index = 0; index < accessor.count; index++) {
            loaderInfo.indexBuffer[loaderInfo.indexPos] = buf[index] + vertexStart;
            loaderInfo.indexPos++;
          }
          break;
        }
        default:
          std::cerr << "Index component type " << accessor.componentType << " not supported!" << std::endl;
          return;
        }
      }
      Primitive *newPrimitive = new Primitive(indexStart, indexCount, vertexCount);
      newMesh->primitives.push_back(newPrimitive);
    }

    newNode->mesh = newMesh;
  }
  if (parent) {
    parent->children.push_back(newNode);
  } else {
    nodes.push_back(newNode);
  }
  linearNodes.push_back(newNode);
}

void Model::getNodeProps(const tinygltf::Node &node, const tinygltf::Model &model, size_t &vertexCount, size_t &indexCount) {
  if (node.children.size() > 0) {
    for (size_t i = 0; i < node.children.size(); i++) {
      getNodeProps(model.nodes[node.children[i]], model, vertexCount, indexCount);
    }
  }
  if (node.mesh > -1) {
    const tinygltf::Mesh mesh = model.meshes[node.mesh];
    for (size_t i = 0; i < mesh.primitives.size(); i++) {
      auto primitive = mesh.primitives[i];
      vertexCount += model.accessors[primitive.attributes.find("POSITION")->second].count;
      if (primitive.indices > -1) {
        indexCount += model.accessors[primitive.indices].count;
      }
    }
  }
}

void Model::loadSkins(tinygltf::Model &gltfModel) {
  for (tinygltf::Skin &source : gltfModel.skins) {
    Skin *newSkin = new Skin{};
    newSkin->name = source.name;

    // Find skeleton root node
    if (source.skeleton > -1) {
      newSkin->skeletonRoot = nodeFromIndex(source.skeleton);
    }

    // Find joint nodes
    for (int jointIndex : source.joints) {
      Node *node = nodeFromIndex(jointIndex);
      if (node) {
        newSkin->joints.push_back(nodeFromIndex(jointIndex));
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

void Model::loadAnimations(tinygltf::Model &gltfModel) {
  for (tinygltf::Animation &anim : gltfModel.animations) {
    vkglTF::Animation animation{};
    animation.name = anim.name;
    if (anim.name.empty()) {
      animation.name = std::to_string(animations.size());
    }

    // Samplers
    for (auto &samp : anim.samplers) {
      vkglTF::AnimationSampler sampler{};

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
      vkglTF::AnimationChannel channel{};

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
      channel.node = nodeFromIndex(source.target_node);
      if (!channel.node) {
        continue;
      }

      animation.channels.push_back(channel);
    }

    animations.push_back(animation);
  }
}

void Model::loadFromFile(std::string filename, Device *device, VkQueue transferQueue, float scale) {
  tinygltf::Model gltfModel;
  tinygltf::TinyGLTF gltfContext;

  std::string error;
  std::string warning;

  this->device = device;

  bool binary = false;
  size_t extpos = filename.rfind('.', filename.length());
  if (extpos != std::string::npos) {
    binary = (filename.substr(extpos + 1, filename.length() - extpos) == "glb");
  }

  bool fileLoaded = binary ? gltfContext.LoadBinaryFromFile(&gltfModel, &error, &warning, filename.c_str())
                           : gltfContext.LoadASCIIFromFile(&gltfModel, &error, &warning, filename.c_str());

  LoaderInfo loaderInfo{};
  size_t vertexCount = 0;
  size_t indexCount = 0;

  if (fileLoaded) {
    const tinygltf::Scene &scene = gltfModel.scenes[0];

    // Get vertex and index buffer sizes up-front
    for (size_t i = 0; i < scene.nodes.size(); i++) {
      getNodeProps(gltfModel.nodes[scene.nodes[i]], gltfModel, vertexCount, indexCount);
    }
    loaderInfo.vertexBuffer = new Vertex[vertexCount];
    loaderInfo.indexBuffer = new uint32_t[indexCount];

    // TODO: scene handling with no default scene
    for (size_t i = 0; i < scene.nodes.size(); i++) {
      const tinygltf::Node node = gltfModel.nodes[scene.nodes[i]];
      loadNode(nullptr, node, scene.nodes[i], gltfModel, loaderInfo, scale);
    }
    if (gltfModel.animations.size() > 0) {
      loadAnimations(gltfModel);
      std::cerr << "animations" << '\n';
    }
    loadSkins(gltfModel);

    for (auto node : linearNodes) {
      // Assign skins
      if (node->skinIndex > -1) {
        node->skin = skins[node->skinIndex];
      }
      // Initial pose
      if (node->mesh) {
        node->update();
      }
    }
  } else {
    // TODO: throw
    std::cerr << "Could not load gltf file: " << error << std::endl;
    return;
  }

  extensions = gltfModel.extensionsUsed;

  size_t vertexBufferSize = vertexCount * sizeof(Vertex);
  size_t indexBufferSize = indexCount * sizeof(uint32_t);
  indices.count = indexCount;

  assert(vertexBufferSize > 0);

  struct StagingBuffer {
    VkBuffer buffer;
    VkDeviceMemory memory;
  } vertexStaging, indexStaging;

  // Create staging buffers
  // Vertex data
  (device->createBuffer(VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, vertexBufferSize,
                        &vertexStaging.buffer, &vertexStaging.memory, loaderInfo.vertexBuffer));
  // Index data
  if (indexBufferSize > 0) {
    (device->createBuffer(VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, indexBufferSize,
                          &indexStaging.buffer, &indexStaging.memory, loaderInfo.indexBuffer));
  }

  // Create device local buffers
  // Vertex buffer
  (device->createBuffer(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertexBufferSize, &vertices.buffer, &vertices.memory));
  // Index buffer
  if (indexBufferSize > 0) {
    (device->createBuffer(VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indexBufferSize, &indices.buffer, &indices.memory));
  }

  // Copy from staging buffers
  VkCommandBuffer copyCmd = device->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

  VkBufferCopy copyRegion = {};

  copyRegion.size = vertexBufferSize;
  vkCmdCopyBuffer(copyCmd, vertexStaging.buffer, vertices.buffer, 1, &copyRegion);

  if (indexBufferSize > 0) {
    copyRegion.size = indexBufferSize;
    vkCmdCopyBuffer(copyCmd, indexStaging.buffer, indices.buffer, 1, &copyRegion);
  }

  device->flushCommandBuffer(copyCmd, transferQueue, true);

  vkDestroyBuffer(device->device, vertexStaging.buffer, nullptr);
  vkFreeMemory(device->device, vertexStaging.memory, nullptr);
  if (indexBufferSize > 0) {
    vkDestroyBuffer(device->device, indexStaging.buffer, nullptr);
    vkFreeMemory(device->device, indexStaging.memory, nullptr);
  }

  delete[] loaderInfo.vertexBuffer;
  delete[] loaderInfo.indexBuffer;
}

void Model::drawNode(Node *node, VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout) {
  if (node->mesh) {
    // Render mesh primitives
    for (vkglTF::Primitive *primitive : node->mesh->primitives) {
      const std::vector<VkDescriptorSet> descriptorsets = {
          node->mesh->uniformBuffer.descriptorSet,
      };
      vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1,
                              static_cast<uint32_t>(descriptorsets.size()), descriptorsets.data(), 0, NULL);

      // TODO: glTF specs states that metallic roughness should be preferred, even if specular glosiness is present
      auto nodeMatrix = node->getMatrix();
      vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4), &nodeMatrix);
      if (primitive->hasIndices) {
        vkCmdDrawIndexed(commandBuffer, primitive->indexCount, 1, primitive->firstIndex, 0, 0);
      } else {
        vkCmdDraw(commandBuffer, primitive->vertexCount, 1, 0, 0);
      }
    }
  };
  for (auto child : node->children) {
    drawNode(child, commandBuffer, pipelineLayout);
  }
}

void Model::draw(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout) {
  const VkDeviceSize offsets[1] = {0};
  vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vertices.buffer, offsets);
  vkCmdBindIndexBuffer(commandBuffer, indices.buffer, 0, VK_INDEX_TYPE_UINT32);
  for (auto &node : nodes) {
    drawNode(node, commandBuffer, pipelineLayout);
  }
}

void Model::updateAnimation(uint32_t index, float time) {
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
    vkglTF::AnimationSampler &sampler = animation.samplers[channel.samplerIndex];
    if (sampler.inputs.size() > sampler.outputsVec4.size()) {
      continue;
    }

    for (size_t i = 0; i < sampler.inputs.size() - 1; i++) {
      if ((animationTimer >= sampler.inputs[i]) && (animationTimer <= sampler.inputs[i + 1])) {
        float u = std::max(0.0f, time - sampler.inputs[i]) / (sampler.inputs[i + 1] - sampler.inputs[i]);
        if (u <= 1.0f) {
          switch (channel.path) {
          case vkglTF::AnimationChannel::PathType::TRANSLATION: {
            glm::vec4 trans = glm::mix(sampler.outputsVec4[i], sampler.outputsVec4[i + 1], u);
            channel.node->translation = glm::vec3(trans);
            break;
          }
          case vkglTF::AnimationChannel::PathType::SCALE: {
            glm::vec4 trans = glm::mix(sampler.outputsVec4[i], sampler.outputsVec4[i + 1], u);
            channel.node->scale = glm::vec3(trans);
            break;
          }
          case vkglTF::AnimationChannel::PathType::ROTATION: {
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

Node *Model::findNode(Node *parent, uint32_t index) {
  Node *nodeFound = nullptr;
  if (parent->index == index) {
    return parent;
  }
  for (auto &child : parent->children) {
    nodeFound = findNode(child, index);
    if (nodeFound) {
      break;
    }
  }
  return nodeFound;
}

Node *Model::nodeFromIndex(uint32_t index) {
  Node *nodeFound = nullptr;
  for (auto &node : nodes) {
    nodeFound = findNode(node, index);
    if (nodeFound) {
      break;
    }
  }
  return nodeFound;
}

} // namespace vkglTF
