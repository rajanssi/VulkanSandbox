find_package(Vulkan REQUIRED)

#file(GLOB PROJECT_HEADERS src/*.h)
#file(GLOB PROJECT_SOURCES src/*.cpp)

set(PROJECT_SOURCES
  Animator.cpp
  Animator.h
  Application.cpp
  Application.h
  Buffer.cpp
  Buffer.h
  Camera.cpp
  Camera.h
  CMakeLists.txt
  Descriptors.cpp
  Descriptors.h
  Device.cpp
  Device.h
  FrameInfo.h
  gltf.cpp
  gltf.h
  Graphics
  main.cpp
  Mesh.cpp
  Mesh.h
  Model.cpp
  Model.h
  Node.cpp
  Node.h
  Pipeline.cpp
  Pipeline.h
  Primitive.cpp
  Primitive.h
  RenderSystem.cpp
  RenderSystem.h
  Skin.h
  SwapChain.cpp
  SwapChain.h
  VulkanBuffer.h
  VulkanTexture.cpp
  VulkanTexture.h
  Window.cpp
  Window.h
  )

set(GRAPHICS_SOURCES
  Graphics/Renderer.cpp
  Graphics/Renderer.h
  )

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${GRAPHICS_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} glfw ${GLFW_LIBRARIES} Vulkan::Vulkan ${CONAN_LIBS})
