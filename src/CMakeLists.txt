find_package(Vulkan REQUIRED)

#file(GLOB PROJECT_HEADERS src/*.h)
#file(GLOB PROJECT_SOURCES src/*.cpp)

set(PROJECT_SOURCES
  Animator.cpp
  Animator.h
  Application.cpp
  Application.h
  Camera.cpp
  Camera.h
  FrameInfo.h
  gltf.cpp
  gltf.h
  main.cpp
  Material.cpp
  Material.h
  Mesh.cpp
  Mesh.h
  Model.cpp
  Model.h
  Node.cpp
  Node.h
  Primitive.cpp
  Primitive.h
  ResourceManager.cpp
  ResourceManager.h
  Resources.cpp
  Resources.h
  Skin.h
  Window.cpp
  Window.h
  )



set(GRAPHICS_SOURCES
  VulkanBackend/Buffer.cpp
  VulkanBackend/Buffer.h
  VulkanBackend/Descriptors.cpp
  VulkanBackend/Descriptors.h
  VulkanBackend/Device.cpp
  VulkanBackend/Device.h
  VulkanBackend/Pipeline.cpp
  VulkanBackend/Pipeline.h
  VulkanBackend/Renderer.cpp
  VulkanBackend/Renderer.h
  VulkanBackend/SwapChain.cpp
  VulkanBackend/SwapChain.h
  VulkanBackend/VulkanBuffer.h
  VulkanBackend/VulkanTexture.cpp
  VulkanBackend/VulkanTexture.h
  )

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${GRAPHICS_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} glfw ${GLFW_LIBRARIES} Vulkan::Vulkan ${CONAN_LIBS})
