#include "Buffer.h"

#include <cassert>
#include <cstring>

Buffer::Buffer(Device &device, VkDeviceSize instanceSize, uint32_t instanceCount, VkBufferUsageFlags usageFlags,
               VkMemoryPropertyFlags memoryPropertyFlags, VkDeviceSize minOffsetAlignment)
    : device{device}, instanceSize{instanceSize}, instanceCount{instanceCount}, usageFlags{usageFlags},
      memoryPropertyFlags{memoryPropertyFlags} {
  alignmentSize =
      (minOffsetAlignment > 0) ? (instanceSize + minOffsetAlignment - 1) & ~(minOffsetAlignment - 1) : instanceSize;
  bufferSize = alignmentSize * instanceCount;
  device.createBuffer(bufferSize, usageFlags, memoryPropertyFlags, buffer, memory);
}

Buffer::Buffer(Device &device, VkDeviceSize size, VkBufferUsageFlags usageFlags, VkMemoryPropertyFlags memoryPropertyFlags,
               void *data)
    : device{device} {
  VkBufferCreateInfo bufferCreateInfo{};
  bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  bufferCreateInfo.usage = usageFlags;
  bufferCreateInfo.size = size;

  vkCreateBuffer(device(), &bufferCreateInfo, nullptr, &buffer);

  VkMemoryRequirements memReqs;
  VkMemoryAllocateInfo memAlloc{};
  memAlloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  vkGetBufferMemoryRequirements(device(), buffer, &memReqs);
  memAlloc.allocationSize = memReqs.size;
  memAlloc.memoryTypeIndex = device.findMemoryType(memReqs.memoryTypeBits, memoryPropertyFlags);
  VkMemoryAllocateFlagsInfoKHR allocFlagsInfo{};
  if (usageFlags & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) {
    allocFlagsInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO_KHR;
    allocFlagsInfo.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT_KHR;
    memAlloc.pNext = &allocFlagsInfo;
  }
  vkAllocateMemory(device(), &memAlloc, nullptr, &memory);

  size = size;
  usageFlags = usageFlags;
  memoryPropertyFlags = memoryPropertyFlags;

  if (data != nullptr) {
    map();
    memcpy(mapped, data, size);
    if ((memoryPropertyFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) == 0)
      flush();

    unmap();
  }

  vkBindBufferMemory(device(), buffer, memory, 0);
}

Buffer::~Buffer() {
  unmap();
  vkDestroyBuffer(device(), buffer, nullptr);
  vkFreeMemory(device(), memory, nullptr);
}

VkResult Buffer::map(VkDeviceSize size, VkDeviceSize offset) {
  assert(buffer && memory && "Called map on buffer before create");
  return vkMapMemory(device(), memory, offset, size, 0, &mapped);
}

void Buffer::unmap() {
  if (mapped) {
    vkUnmapMemory(device(), memory);
    mapped = nullptr;
  }
}

void Buffer::writeToBuffer(void *data, VkDeviceSize size, VkDeviceSize offset) {
  assert(mapped && "Cannot copy to unmapped buffer");

  if (size == VK_WHOLE_SIZE) {
    memcpy(mapped, data, bufferSize);
  } else {
    char *memOffset = (char *)mapped;
    memOffset += offset;
    memcpy(memOffset, data, size);
  }
}

VkResult Buffer::flush(VkDeviceSize size, VkDeviceSize offset) {
  VkMappedMemoryRange mappedRange = {};
  mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
  mappedRange.memory = memory;
  mappedRange.offset = offset;
  mappedRange.size = size;
  return vkFlushMappedMemoryRanges(device(), 1, &mappedRange);
}

VkDescriptorBufferInfo Buffer::descriptorInfo(VkDeviceSize size, VkDeviceSize offset) {
  return VkDescriptorBufferInfo{
      buffer,
      offset,
      size,
  };
}
