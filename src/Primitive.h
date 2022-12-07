#pragma once

#include <cstdint>

struct Primitive {
  uint32_t firstIndex;
  uint32_t indexCount;
  uint32_t vertexCount;
  Primitive(uint32_t firstIndex, uint32_t indexCount, uint32_t vertexCount);
};
