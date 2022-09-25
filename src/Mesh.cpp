#include "Mesh.h"

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
