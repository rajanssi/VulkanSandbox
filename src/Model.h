#pragma once

#include "Buffer.h"
#include "Device.h"
#include "VulkanTexture.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <tiny_gltf.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <memory>
#include <functional>

class Model {
public:
  struct Vertex {
    glm::vec3 position{};
    glm::vec3 color{};
    glm::vec3 normal{};
    glm::vec2 uv{};

    static std::vector<VkVertexInputBindingDescription> getBindingDescriptions();
    static std::vector<VkVertexInputAttributeDescription> getAttributeDescriptions();

    // NOTE: new for skinning
    glm::vec4 jointIndices;
    glm::vec4 jointWeights;
  };

  struct Node;

  struct Primitive {
    uint32_t firstIndex;
    uint32_t indexCount;
    int32_t materialIndex;
  };

  struct Mesh {
    std::vector<Primitive> primitives;
  };

  struct Node {
    Node *parent;
    std::vector<Node *> children;
    Mesh mesh;

    glm::mat4 matrix;
    // NOTE: new for skinning
    uint32_t index;
    glm::vec3 translation{};
    glm::vec3 scale{1.0f};
    glm::quat rotation{};
    int32_t skin = -1;
    glm::mat4 getLocalMatrix();
  };

  struct Material {
    glm::vec4 baseColorFactor = glm::vec4(1.0f);
    uint32_t baseColorTextureIndex;
  };

  struct Image {
    Texture2D texture;
    VkDescriptorSet descriptorSet;
  };

  struct Texture {
    int32_t imageIndex;
  };

  struct Skin {
    std::string name;
    Node *skeletonRoot = nullptr;
    std::vector<glm::mat4> inverseBindMatrices;
    std::vector<Node *> joints;
    std::unique_ptr<Buffer> ssbo = nullptr;
    VkDescriptorSet descriptorSet;
  };

  struct AnimationSampler {
    std::string interpolation;
    std::vector<float> inputs;
    std::vector<glm::vec4> outputsVec4;
  };

  struct AnimationChannel {
    std::string path;
    Node *node;
    uint32_t samplerIndex;
  };

  struct Animation {
    std::string name;
    std::vector<AnimationSampler> samplers;
    std::vector<AnimationChannel> channels;
    float start = std::numeric_limits<float>::max();
    float end = std::numeric_limits<float>::min();
    float currentTime = 0.0f;
  };

  std::vector<Image> images;
  std::vector<Texture> textures;
  std::vector<Material> materials;
  std::vector<Node *> nodes;
  // NOTE: new for animation
  std::vector<Skin> skins;
  std::vector<Animation> animations;

  uint32_t activeAnimation = 0;

  struct Builder {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    void loadglTFModel(const std::string &filePath, std::vector<Node *> &nodes, std::vector<Image> &images, Device &device,
                       std::vector<Material> &materials, std::vector<Texture> &textures, std::vector<Skin> &skins,
                       std::vector<Animation> &animations);
    void loadNode(const tinygltf::Node &node, const tinygltf::Model &model, Model::Node *parent, uint32_t nodeIndex,
                  std::vector<uint32_t> &indices, std::vector<Vertex> &vertices, std::vector<Node *> &nodes);
    void loadImages(tinygltf::Model &input, std::vector<Image> &images, Device &device);
    void loadMaterials(tinygltf::Model &input, std::vector<Material> &materials);
    void loadTextures(tinygltf::Model &input, std::vector<Texture> &textures);


    void loadSkins(tinygltf::Model &input, std::vector<Skin> &skins, Device &device, std::vector<Node *> &nodes);
    void loadAnimations(tinygltf::Model &input, std::vector<Animation> &animations, std::vector<Node *> &nodes);
    Node *nodeFromIndex(uint32_t index, std::vector<Node *> &nodes);
    Node *findNode(Node *parent, uint32_t index);
  };

  Model(Device &device, const Builder &builder, std::vector<Node *> &nodes, std::vector<Image> &images,
        std::vector<Material> &materials, std::vector<Texture> &textures, std::vector<Skin> &skins,
        std::vector<Animation> &animations);
  ~Model();

  Model(const Model &) = delete;
  Model &operator=(const Model &) = delete;

  void updateAnimations(float deltaTime);
  void draw(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout = VK_NULL_HANDLE);
  void bind(VkCommandBuffer commandBuffer);
  glm::vec3 color() { return {.9f, .6f, .1f}; };
  static std::unique_ptr<Model> createglTFModel(Device &device, const std::string &filePath);

private:
  Device &device;

  std::unique_ptr<Buffer> vertexBuffer;
  uint32_t vertexCount;

  std::unique_ptr<Buffer> indexBuffer;
  uint32_t indexCount;

  void createVertexBuffers(const std::vector<Vertex> &vertices);
  void drawNode(VkCommandBuffer commandBuffer, VkPipelineLayout pipelineLayout, Node node);
  void createIndexBuffers(const std::vector<uint32_t> &indices);
  void updateJoints(Node *node);
  glm::mat4 getNodeMatrix(Node *node);
};
