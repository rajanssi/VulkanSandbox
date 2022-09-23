#include "Model.h"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string.h>

#include "gltf.h"

std::vector<VkVertexInputBindingDescription> Model::Vertex::getBindingDescriptions() {
  std::vector<VkVertexInputBindingDescription> bindingDescriptions(1);
  bindingDescriptions[0].binding = 0;
  bindingDescriptions[0].stride = sizeof(Vertex);
  bindingDescriptions[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
  return bindingDescriptions;
}

std::vector<VkVertexInputAttributeDescription> Model::Vertex::getAttributeDescriptions() {
  std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};

  attributeDescriptions.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, position)});
  attributeDescriptions.push_back({1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, normal)});
  attributeDescriptions.push_back({2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, uv)});
  attributeDescriptions.push_back({3, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, color)});
  attributeDescriptions.push_back({4, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(Vertex, jointIndices)});
  attributeDescriptions.push_back({5, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(Vertex, jointWeights)});

  return attributeDescriptions;
}

Model::Model(Device &device, const Builder &builder, std::vector<Node *> &nodes, std::vector<Image> &images,
             std::vector<Material> &materials, std::vector<Texture> &textures, std::vector<Skin> &skins,
             std::vector<Animation> &animations)
    : device{device} {
  this->nodes = std::move(nodes);
  this->images = std::move(images);
  this->materials = std::move(materials);
  this->textures = std::move(textures);
  this->skins = std::move(skins);
  this->animations = std::move(animations);
  for (auto node : nodes) {
    // updateJoints(node);
  }

  createVertexBuffers(builder.vertices);
  createIndexBuffers(builder.indices);
}

Model::~Model() {
  // NOTE: These are from Sascha Willems samples
  for (Image image : images) {
    vkDestroyImageView(device(), image.texture.view, nullptr);
    vkDestroyImage(device(), image.texture.image, nullptr);
    vkDestroySampler(device(), image.texture.sampler, nullptr);
    vkFreeMemory(device(), image.texture.deviceMemory, nullptr);
  }
}

void Model::createVertexBuffers(const std::vector<Vertex> &vertices) {
  vertexCount = static_cast<uint32_t>(vertices.size());
  assert(vertexCount >= 3 && "Vertex count must be at least 3");
  VkDeviceSize bufferSize = sizeof(vertices[0]) * vertexCount;
  uint32_t vertexSize = sizeof(vertices[0]);

  Buffer stagingBuffer{device, vertexSize, vertexCount, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};

  stagingBuffer.map();
  stagingBuffer.writeToBuffer((void *)vertices.data());

  vertexBuffer = std::make_unique<Buffer>(device, vertexSize, vertexCount,
                                          VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

  device.copyBuffer(stagingBuffer.getBuffer(), vertexBuffer->getBuffer(), bufferSize);
}

void Model::createIndexBuffers(const std::vector<uint32_t> &indices) {
  indexCount = static_cast<uint32_t>(indices.size());
  assert(indexCount >= 3 && "Vertex count must be at least 3");
  VkDeviceSize bufferSize = sizeof(indices[0]) * indexCount;
  uint32_t indexSize = sizeof(indices[0]);

  Buffer stagingBuffer{device, indexSize, indexCount, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};

  stagingBuffer.map();
  stagingBuffer.writeToBuffer((void *)indices.data());

  indexBuffer = std::make_unique<Buffer>(device, indexSize, indexCount,
                                         VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

  device.copyBuffer(stagingBuffer.getBuffer(), indexBuffer->getBuffer(), bufferSize);
}

void Model::draw(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout) {
  if (nodes.empty()) {
    std::cout << "!" << '\n';
    vkCmdDrawIndexed(commandBuffer, indexCount, 1, 0, 0, 0);
  } else {
    for (auto &node : nodes) {
      drawNode(commandBuffer, pipelineLayout, *node);
    }
  }
}

void Model::drawNode(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout, Node node) {
  if (node.mesh.primitives.size() > 0) {
    glm::mat4 nodeMatrix = node.matrix;
    Node *currentParent = node.parent;
    while (currentParent) {
      nodeMatrix = currentParent->matrix * nodeMatrix;
      currentParent = currentParent->parent;
    }

    // Pash final matrix to vertex shader via push constants
    vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4), &nodeMatrix);
    // Bind SSBO with skin data
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, 1,
                            &skins[node.skin].descriptorSet, 0, nullptr);

    for (Primitive &primitive : node.mesh.primitives) {
      if (primitive.indexCount > 0) {
        // Get the texture index for this primitive
        Texture texture = textures[materials[primitive.materialIndex].baseColorTextureIndex];

        // Bind the descriptor for the current primitive's texture
        vkCmdDrawIndexed(commandBuffer, primitive.indexCount, 1, primitive.firstIndex, 0, 0);
      }
    }
  }
  for (auto &child : node.children) {
    drawNode(commandBuffer, pipelineLayout, *child);
  }
}

void Model::bind(VkCommandBuffer commandBuffer) {
  VkBuffer buffers[] = {vertexBuffer->getBuffer()};
  VkDeviceSize offsets[] = {0};
  vkCmdBindVertexBuffers(commandBuffer, 0, 1, buffers, offsets);
  vkCmdBindIndexBuffer(commandBuffer, indexBuffer->getBuffer(), 0, VK_INDEX_TYPE_UINT32);
}

std::unique_ptr<Model> Model::createglTFModel(Device &device, const std::string &filePath) {
  Builder builder{};

  std::vector<Node *> nodes;
  std::vector<Image> images;
  std::vector<Material> materials;
  std::vector<Texture> textures;
  std::vector<Skin> skins;
  std::vector<Animation> animations;

  builder.loadglTFModel(filePath, nodes, images, device, materials, textures, skins, animations);
  std::cout << "Vertex count: " << builder.vertices.size() << '\n';
  assert((uint32_t)builder.vertices.size() >= 3 && "Must be at least 3 vertices");

  return std::make_unique<Model>(device, builder, nodes, images, materials, textures, skins, animations);
}

// NOTE:Vulkan Samples functions
// TODO:Vulkan Samples functions
// NOTE:Vulkan Samples functions
//
void Model::Builder::loadglTFModel(const std::string &filePath, std::vector<Node *> &nodes, std::vector<Image> &images,
                                   Device &device, std::vector<Material> &materials, std::vector<Texture> &textures,
                                   std::vector<Skin> &skins, std::vector<Animation> &animations) {
  tinygltf::Model glTFInput;
  tinygltf::TinyGLTF gltfContext;
  std::string err, warn;

  bool fileLoaded = gltfContext.LoadASCIIFromFile(&glTFInput, &err, &warn, filePath);

  if (fileLoaded) {
    loadImages(glTFInput, images, device);
    loadMaterials(glTFInput, materials);
    loadTextures(glTFInput, textures);
    const tinygltf::Scene &scene = glTFInput.scenes[0];
    for (size_t i = 0; i < scene.nodes.size(); ++i) {
      const tinygltf::Node node = glTFInput.nodes[scene.nodes[i]];
      loadNode(node, glTFInput, nullptr, scene.nodes[i], indices, vertices, nodes);
    }
    loadSkins(glTFInput, skins, device, nodes);
    loadAnimations(glTFInput, animations, nodes);
  } else {
    throw std::runtime_error(err);
  }
}

void Model::Builder::loadImages(tinygltf::Model &input, std::vector<Image> &images, Device &device) {
  // Images can be stored inside the glTF (which is the case for the sample model), so instead of directly
  // loading them from disk, we fetch them from the glTF loader and upload the buffers
  images.resize(input.images.size());
  for (size_t i = 0; i < input.images.size(); i++) {
    tinygltf::Image &glTFImage = input.images[i];
    // Get the image data from the glTF loader
    unsigned char *buffer = nullptr;
    VkDeviceSize bufferSize = 0;
    bool deleteBuffer = false;
    // We convert RGB-only images to RGBA, as most devices don't support RGB-formats in Vulkan
    if (glTFImage.component == 3) {
      bufferSize = glTFImage.width * glTFImage.height * 4;
      buffer = new unsigned char[bufferSize];
      unsigned char *rgba = buffer;
      unsigned char *rgb = &glTFImage.image[0];
      for (size_t i = 0; i < glTFImage.width * glTFImage.height; ++i) {
        memcpy(rgba, rgb, sizeof(unsigned char) * 3);
        rgba += 4;
        rgb += 3;
      }
      deleteBuffer = true;
    } else {
      buffer = &glTFImage.image[0];
      bufferSize = glTFImage.image.size();
    }
    // Load texture from image buffer
    images[i].texture.fromBuffer(buffer, bufferSize, VK_FORMAT_R8G8B8A8_UNORM, glTFImage.width, glTFImage.height, &device);
    if (deleteBuffer) {
      delete[] buffer;
    }
  }
}

void Model::Builder::loadMaterials(tinygltf::Model &input, std::vector<Material> &materials) {
  materials.resize(input.materials.size());
  for (size_t i = 0; i < input.materials.size(); i++) {
    // We only read the most basic properties required for our sample
    tinygltf::Material glTFMaterial = input.materials[i];
    // Get the base color factor
    if (glTFMaterial.values.find("baseColorFactor") != glTFMaterial.values.end()) {
      materials[i].baseColorFactor = glm::make_vec4(glTFMaterial.values["baseColorFactor"].ColorFactor().data());
    }
    // Get base color texture index
    if (glTFMaterial.values.find("baseColorTexture") != glTFMaterial.values.end()) {
      materials[i].baseColorTextureIndex = glTFMaterial.values["baseColorTexture"].TextureIndex();
    }
  }
}

void Model::Builder::loadTextures(tinygltf::Model &input, std::vector<Texture> &textures) {
  textures.resize(input.textures.size());
  for (size_t i = 0; i < input.textures.size(); i++) {
    textures[i].imageIndex = input.textures[i].source;
  }
}

void Model::Builder::loadNode(const tinygltf::Node &inputNode, const tinygltf::Model &input, Model::Node *parent,
                              uint32_t nodeIndex, std::vector<uint32_t> &indices, std::vector<Vertex> &vertices,
                              std::vector<Node *> &nodes) {
  Model::Node *node = new Node{};
  node->parent = parent;
  node->matrix = glm::mat4(1.0f);
  node->index = nodeIndex;
  node->skin = inputNode.skin;

  // Get the local node matrix
  // It's either made up from translation, rotation, scale or a 4x4 matrix
  if (inputNode.translation.size() == 3) {
    node->translation = glm::make_vec3(inputNode.translation.data());
  }
  if (inputNode.rotation.size() == 4) {
    glm::quat q = glm::make_quat(inputNode.rotation.data());
    node->rotation = glm::mat4(q);
  }
  if (inputNode.scale.size() == 3) {
    node->scale = glm::make_vec3(inputNode.scale.data());
  }
  if (inputNode.matrix.size() == 16) {
    node->matrix = glm::make_mat4x4(inputNode.matrix.data());
  };

  if (inputNode.children.size() > 0) {
    for (size_t i = 0; i < inputNode.children.size(); i++) {
      loadNode(input.nodes[inputNode.children[i]], input, node, inputNode.children[i], indices, vertices, nodes);
    }
  }

  if (inputNode.mesh > -1) {
    const tinygltf::Mesh mesh = input.meshes[inputNode.mesh];
    // Iterate through all primitives of this node's mesh
    for (size_t i = 0; i < mesh.primitives.size(); i++) {
      const tinygltf::Primitive &glTFPrimitive = mesh.primitives[i];
      uint32_t firstIndex = static_cast<uint32_t>(indices.size());
      uint32_t vertexStart = static_cast<uint32_t>(vertices.size());
      uint32_t indexCount = 0;

      // Vertices
      {
        const float *positionBuffer = nullptr;
        const float *normalsBuffer = nullptr;
        const float *texCoordsBuffer = nullptr;
        const uint16_t *jointIndicesBuffer = nullptr;
        const float *jointWeightsBuffer = nullptr;
        size_t vertexCount = 0;

        // Get buffer data for vertex positions
        if (glTFPrimitive.attributes.find("POSITION") != glTFPrimitive.attributes.end()) {
          const tinygltf::Accessor &accessor = input.accessors[glTFPrimitive.attributes.find("POSITION")->second];
          const tinygltf::BufferView &view = input.bufferViews[accessor.bufferView];
          positionBuffer =
              reinterpret_cast<const float *>(&(input.buffers[view.buffer].data[accessor.byteOffset + view.byteOffset]));
          vertexCount = accessor.count;
        }
        // Get buffer data for vertex normals
        if (glTFPrimitive.attributes.find("NORMAL") != glTFPrimitive.attributes.end()) {
          const tinygltf::Accessor &accessor = input.accessors[glTFPrimitive.attributes.find("NORMAL")->second];
          const tinygltf::BufferView &view = input.bufferViews[accessor.bufferView];
          normalsBuffer =
              reinterpret_cast<const float *>(&(input.buffers[view.buffer].data[accessor.byteOffset + view.byteOffset]));
        }
        // Get buffer data for vertex texture coordinates
        // glTF supports multiple sets, we only load the first one
        if (glTFPrimitive.attributes.find("TEXCOORD_0") != glTFPrimitive.attributes.end()) {
          const tinygltf::Accessor &accessor = input.accessors[glTFPrimitive.attributes.find("TEXCOORD_0")->second];
          const tinygltf::BufferView &view = input.bufferViews[accessor.bufferView];
          texCoordsBuffer =
              reinterpret_cast<const float *>(&(input.buffers[view.buffer].data[accessor.byteOffset + view.byteOffset]));
        }
        // NOTE: new for animations
        if (glTFPrimitive.attributes.find("JOINTS_0") != glTFPrimitive.attributes.end()) {
          const tinygltf::Accessor &accessor = input.accessors[glTFPrimitive.attributes.find("JOINTS_0")->second];
          const tinygltf::BufferView &view = input.bufferViews[accessor.bufferView];
          jointIndicesBuffer =
              reinterpret_cast<const uint16_t *>(&(input.buffers[view.buffer].data[accessor.byteOffset + view.byteOffset]));
        }
        if (glTFPrimitive.attributes.find("WEIGHTS_0") != glTFPrimitive.attributes.end()) {
          const tinygltf::Accessor &accessor = input.accessors[glTFPrimitive.attributes.find("WEIGHTS_0")->second];
          const tinygltf::BufferView &view = input.bufferViews[accessor.bufferView];
          jointWeightsBuffer =
              reinterpret_cast<const float *>(&(input.buffers[view.buffer].data[accessor.byteOffset + view.byteOffset]));
        }

        bool hasSkin = (jointIndicesBuffer && jointWeightsBuffer);

        // Append data to model's vertex buffer
        for (size_t v = 0; v < vertexCount; v++) {
          Vertex vertex{};
          vertex.position = glm::vec4(glm::make_vec3(&positionBuffer[v * 3]), 1.0f);
          vertex.normal = glm::normalize(glm::vec3(normalsBuffer ? glm::make_vec3(&normalsBuffer[v * 3]) : glm::vec3(0.0f)));
          vertex.uv = texCoordsBuffer ? glm::make_vec2(&texCoordsBuffer[v * 2]) : glm::vec3(0.0f);
          vertex.color = glm::vec3(1.0f);
          vertex.jointIndices = hasSkin ? glm::vec4(glm::make_vec4(&jointIndicesBuffer[v * 4])) : glm::vec4(0.0f);
          vertex.jointWeights = hasSkin ? glm::vec4(glm::make_vec4(&jointWeightsBuffer[v * 4])) : glm::vec4(0.0f);
          vertices.push_back(vertex);
        }
      }
      // Indices
      {
        const tinygltf::Accessor &accessor = input.accessors[glTFPrimitive.indices];
        const tinygltf::BufferView &bufferView = input.bufferViews[accessor.bufferView];
        const tinygltf::Buffer &buffer = input.buffers[bufferView.buffer];

        indexCount += static_cast<uint32_t>(accessor.count);

        // glTF supports different component types of indices
        switch (accessor.componentType) {
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_INT: {
          const uint32_t *buf = reinterpret_cast<const uint32_t *>(&buffer.data[accessor.byteOffset + bufferView.byteOffset]);
          for (size_t index = 0; index < accessor.count; index++) {
            indices.push_back(buf[index] + vertexStart);
          }
          break;
        }
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_SHORT: {
          const uint16_t *buf = reinterpret_cast<const uint16_t *>(&buffer.data[accessor.byteOffset + bufferView.byteOffset]);
          for (size_t index = 0; index < accessor.count; index++) {
            indices.push_back(buf[index] + vertexStart);
          }
          break;
        }
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_BYTE: {
          const uint8_t *buf = reinterpret_cast<const uint8_t *>(&buffer.data[accessor.byteOffset + bufferView.byteOffset]);
          for (size_t index = 0; index < accessor.count; index++) {
            indices.push_back(buf[index] + vertexStart);
          }
          break;
        }
        default:
          std::cerr << "Index component type " << accessor.componentType << " not supported!" << std::endl;
          return;
        }
      }
      Primitive primitive{};
      primitive.firstIndex = firstIndex;
      primitive.indexCount = indexCount;
      primitive.materialIndex = glTFPrimitive.material;
      node->mesh.primitives.push_back(primitive);
    }
  }
  if (parent) {
    parent->children.push_back(node);
  } else {
    nodes.push_back(node);
  }
}

glm::mat4 Model::getNodeMatrix(Node *node) {
  glm::mat4 nodeMatrix = node->getLocalMatrix();
  Node *currentParent = node->parent;
  while (currentParent) {
    nodeMatrix = currentParent->getLocalMatrix() * nodeMatrix;
    currentParent = currentParent->parent;
  }
  return nodeMatrix;
}

void Model::updateJoints(Node *node) {
  if (node->skin > -1) {
    // Update the joint matrices
    glm::mat4 inverseTransform = glm::inverse(getNodeMatrix(node));
    Skin &skin = skins[node->skin];
    size_t numJoints = (uint32_t)skin.joints.size();
    std::vector<glm::mat4> jointMatrices(numJoints);
    for (size_t i = 0; i < numJoints; i++) {
      jointMatrices[i] = getNodeMatrix(skin.joints[i]) * skin.inverseBindMatrices[i];
      jointMatrices[i] = inverseTransform * jointMatrices[i];
    }
    // Update ssbo
    skin.ssbo->writeToBuffer(jointMatrices.data(), jointMatrices.size() * sizeof(glm::mat4));
  }

  for (auto &child : node->children) {
    updateJoints(child);
  }
}

void Model::updateAnimations(float deltaTime) {
  if (activeAnimation > static_cast<uint32_t>(animations.size()) - 1) {
    std::cout << "No animation with index " << activeAnimation << std::endl;
    return;
  }
  Animation &animation = animations[activeAnimation];
  animation.currentTime += deltaTime;
  if (animation.currentTime > animation.end) {
    animation.currentTime -= animation.end;
  }

  for (auto &channel : animation.channels) {
    AnimationSampler &sampler = animation.samplers[channel.samplerIndex];
    for (size_t i = 0; i < sampler.inputs.size() - 1; i++) {
      if (sampler.interpolation != "LINEAR") {
        std::cout << "This sample only supports linear interpolations\n";
        continue;
      }

      // Get the input keyframe values for the current time stamp
      if ((animation.currentTime >= sampler.inputs[i]) && (animation.currentTime <= sampler.inputs[i + 1])) {
        float a = (animation.currentTime - sampler.inputs[i]) / (sampler.inputs[i + 1] - sampler.inputs[i]);
        if (channel.path == "translation") {
          channel.node->translation = glm::mix(sampler.outputsVec4[i], sampler.outputsVec4[i + 1], a);
        }
        if (channel.path == "rotation") {
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

          channel.node->rotation = glm::normalize(glm::slerp(q1, q2, a));
        }
        if (channel.path == "scale") {
          channel.node->scale = glm::mix(sampler.outputsVec4[i], sampler.outputsVec4[i + 1], a);
        }
      }
    }
  }
  for (auto &node : nodes) {
    updateJoints(node);
  }
}

Model::Node *Model::Builder::findNode(Node *parent, uint32_t index) {
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

Model::Node *Model::Builder::nodeFromIndex(uint32_t index, std::vector<Node *> &nodes) {
  Node *nodeFound = nullptr;
  for (auto &node : nodes) {
    nodeFound = findNode(node, index);
    if (nodeFound) {
      break;
    }
  }
  return nodeFound;
}

void Model::Builder::loadAnimations(tinygltf::Model &input, std::vector<Animation> &animations, std::vector<Node *> &nodes) {
  animations.resize(input.animations.size());

  for (size_t i = 0; i < input.animations.size(); i++) {
    tinygltf::Animation glTFAnimation = input.animations[i];
    animations[i].name = glTFAnimation.name;

    // Samplers
    animations[i].samplers.resize(glTFAnimation.samplers.size());
    for (size_t j = 0; j < glTFAnimation.samplers.size(); j++) {
      tinygltf::AnimationSampler glTFSampler = glTFAnimation.samplers[j];
      AnimationSampler &dstSampler = animations[i].samplers[j];
      dstSampler.interpolation = glTFSampler.interpolation;

      // Read sampler keyframe input time values
      {
        const tinygltf::Accessor &accessor = input.accessors[glTFSampler.input];
        const tinygltf::BufferView &bufferView = input.bufferViews[accessor.bufferView];
        const tinygltf::Buffer &buffer = input.buffers[bufferView.buffer];
        const void *dataPtr = &buffer.data[accessor.byteOffset + bufferView.byteOffset];
        const float *buf = static_cast<const float *>(dataPtr);
        for (size_t index = 0; index < accessor.count; index++) {
          dstSampler.inputs.push_back(buf[index]);
        }
        // Adjust animation's start and end times
        for (auto input : animations[i].samplers[j].inputs) {
          if (input < animations[i].start) {
            animations[i].start = input;
          };
          if (input > animations[i].end) {
            animations[i].end = input;
          }
        }
      }

      // Read sampler keyframe output translate/rotate/scale values
      {
        const tinygltf::Accessor &accessor = input.accessors[glTFSampler.output];
        const tinygltf::BufferView &bufferView = input.bufferViews[accessor.bufferView];
        const tinygltf::Buffer &buffer = input.buffers[bufferView.buffer];
        const void *dataPtr = &buffer.data[accessor.byteOffset + bufferView.byteOffset];
        switch (accessor.type) {
        case TINYGLTF_TYPE_VEC3: {
          const glm::vec3 *buf = static_cast<const glm::vec3 *>(dataPtr);
          for (size_t index = 0; index < accessor.count; index++) {
            dstSampler.outputsVec4.push_back(glm::vec4(buf[index], 0.0f));
          }
          break;
        }
        case TINYGLTF_TYPE_VEC4: {
          const glm::vec4 *buf = static_cast<const glm::vec4 *>(dataPtr);
          for (size_t index = 0; index < accessor.count; index++) {
            dstSampler.outputsVec4.push_back(buf[index]);
          }
          break;
        }
        default: {
          std::cout << "unknown type" << std::endl;
          break;
        }
        }
      }
    }

    // Channels
    animations[i].channels.resize(glTFAnimation.channels.size());
    for (size_t j = 0; j < glTFAnimation.channels.size(); j++) {
      tinygltf::AnimationChannel glTFChannel = glTFAnimation.channels[j];
      AnimationChannel &dstChannel = animations[i].channels[j];
      dstChannel.path = glTFChannel.target_path;
      dstChannel.samplerIndex = glTFChannel.sampler;
      dstChannel.node = nodeFromIndex(glTFChannel.target_node, nodes);
    }
  }
}

void Model::Builder::loadSkins(tinygltf::Model &input, std::vector<Skin> &skins, Device &device, std::vector<Node *> &nodes) {
  skins.resize(input.skins.size());

  for (size_t i = 0; i < input.skins.size(); i++) {
    tinygltf::Skin glTFSkin = input.skins[i];
    skins[i].name = glTFSkin.name;
    // Find the root node of the skeleton
    skins[i].skeletonRoot = nodeFromIndex(glTFSkin.skeleton, nodes);

    // Find joint nodes
    for (int jointIndex : glTFSkin.joints) {
      Node *node = nodeFromIndex(jointIndex, nodes);

      if (node) {
        skins[i].joints.push_back(node);
      }
    }

    if (glTFSkin.inverseBindMatrices > -1) {
      const tinygltf::Accessor &accessor = input.accessors[glTFSkin.inverseBindMatrices];
      const tinygltf::BufferView &bufferView = input.bufferViews[accessor.bufferView];
      const tinygltf::Buffer &buffer = input.buffers[bufferView.buffer];
      skins[i].inverseBindMatrices.resize(accessor.count);
      memcpy(skins[i].inverseBindMatrices.data(), &buffer.data[accessor.byteOffset + bufferView.byteOffset],
             accessor.count * sizeof(glm::mat4));

      skins[i].ssbo = std::make_unique<Buffer>(
          device, sizeof(glm::mat4) * skins[i].inverseBindMatrices.size(), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, skins[i].inverseBindMatrices.data());
      skins[i].ssbo->map();
    }
  }
}

glm::mat4 Model::Node::getLocalMatrix() {
  return glm::translate(glm::mat4(1.0f), translation) * glm::mat4(rotation) * glm::scale(glm::mat4(1.0f), scale) * matrix;
}
