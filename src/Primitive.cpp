#include "Primitive.h"

Primitive::Primitive(uint32_t firstIndex, uint32_t indexCount, uint32_t vertexCount)
    : firstIndex(firstIndex), indexCount(indexCount), vertexCount(vertexCount) {
  hasIndices = indexCount > 0;
};
