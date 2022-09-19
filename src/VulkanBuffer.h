#pragma once

#include "Device.h"

struct VulkanBuffer {
  VkDevice device;
  VkBuffer buffer = VK_NULL_HANDLE;
  VkDeviceMemory memory = VK_NULL_HANDLE;
  VkDescriptorBufferInfo descriptor;
  int32_t count = 0;
  void *mapped = nullptr;
  void create(Device *device, VkBufferUsageFlags usageFlags, VkMemoryPropertyFlags memoryPropertyFlags,
              VkDeviceSize size, bool map = true) {
    this->device = device->device;
    device->createBuffer(usageFlags, memoryPropertyFlags, size, &buffer, &memory);
    descriptor = {buffer, 0, size};
    if (map) {
      (vkMapMemory(device->device, memory, 0, size, 0, &mapped));
    }
  }
  void destroy() {
    if (mapped) {
      unmap();
    }
    vkDestroyBuffer(device, buffer, nullptr);
    vkFreeMemory(device, memory, nullptr);
    buffer = VK_NULL_HANDLE;
    memory = VK_NULL_HANDLE;
  }
  void map() { (vkMapMemory(device, memory, 0, VK_WHOLE_SIZE, 0, &mapped)); }
  void unmap() {
    if (mapped) {
      vkUnmapMemory(device, memory);
      mapped = nullptr;
    }
  }
  void flush(VkDeviceSize size = VK_WHOLE_SIZE) {
    VkMappedMemoryRange mappedRange{};
    mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    mappedRange.memory = memory;
    mappedRange.size = size;
    (vkFlushMappedMemoryRanges(device, 1, &mappedRange));
  }
};
