cmake_minimum_required(VERSION 3.21)
project(VulkanSandbox)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

option(GLFW_BUILD_DOCS OFF)
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)
add_subdirectory(external/glfw)

SET(GLM_TEST_ENABLE OFF CACHE BOOL "GLM Build unit tests")
include_directories(external/glm)

add_subdirectory(src)

file(GLOB_RECURSE PROJECT_SHADERS
  data/shaders/*.vert
  data/shaders/*.frag)
set(SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/data/shaders)
set(SHADER_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/data/shaders)
set(GLSL_VALIDATOR "glslc")

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

  #add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  #COMMAND ${CMAKE_COMMAND} -E copy_directory
  #"${PROJECT_BINARY_DIR}/data/shaders"
  #    "$<TARGET_FILE_DIR:${PROJECT_NAME}>/data/shaders"
  #    )

