#pragma once

#include <string>

#define GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

class Window {
public:
  bool resized = false;

  Window(int width, int height, const std::string title);
  ~Window();
  Window(const Window &) = delete;
  Window &operator=(const Window &) = delete;

  GLFWwindow *getWindow() const { return window; };
  bool close() { return glfwWindowShouldClose(window); };
  VkExtent2D extent() { return {static_cast<uint32_t>(width), static_cast<uint32_t>(height)}; };
  void createSurface(VkInstance instance, VkSurfaceKHR *surface);
private:
  int width, height;
  GLFWwindow *window;
  const std::string title;

  static void framebufferSizeCallback(GLFWwindow *window, int width, int height);
};
