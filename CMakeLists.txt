cmake_minimum_required(VERSION 3.21)
project(VulkanSandbox)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

find_package(Vulkan REQUIRED)

option(GLFW_BUILD_DOCS OFF)
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)
add_subdirectory(external/glfw)

SET(GLM_TEST_ENABLE OFF CACHE BOOL "GLM Build unit tests")
include_directories(external/glm)

set(TINYGLTF_HEADER_ONLY ON CACHE INTERNAL "" FORCE)
set(TINYGLTF_INSTALL OFF CACHE INTERNAL "" FORCE)
add_subdirectory(external/tinygltf)

set(SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/data/shaders)
set(SHADER_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/data/shaders)
set(GLSL_VALIDATOR "glslc")

file(GLOB PROJECT_HEADERS src/*.h)
file(GLOB PROJECT_SOURCES src/*.cpp)
file(GLOB_RECURSE PROJECT_SHADERS
  data/shaders/*.vert
  data/shaders/*.frag)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${PROJECT_HEADERS})

target_link_libraries(${PROJECT_NAME} glfw ${GLFW_LIBRARIES} Vulkan::Vulkan tinygltf ${TINYGLTF_LIBRARIES})

foreach(GLSL ${PROJECT_SHADERS})
 get_filename_component(FILE_NAME ${GLSL} NAME)
 set(SPIRV "${PROJECT_BINARY_DIR}/data/shaders/${FILE_NAME}.spv")
 add_custom_command(
   OUTPUT ${SPIRV}
   COMMAND ${CMAKE_COMMAND} -E make_directory "${PROJECT_BINARY_DIR}/data/shaders/"
   COMMAND ${GLSL_VALIDATOR} ${GLSL} -o ${SPIRV}
   DEPENDS ${GLSL})
 list(APPEND SPIRV_BINARY_FILES ${SPIRV})
endforeach(GLSL)

add_custom_target(
    Shaders
    DEPENDS ${SPIRV_BINARY_FILES}
    )

  add_dependencies(${PROJECT_NAME} Shaders)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${PROJECT_BINARY_DIR}/data/shaders"
      "$<TARGET_FILE_DIR:${PROJECT_NAME}>/data/shaders"
      )

