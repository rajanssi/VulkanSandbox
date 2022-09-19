#include "Window.h"

#include <stdexcept>

namespace {
bool firstMouse = true;
float lastX, lastY;
} // namespace

Window::Window(int width, int height, const std::string title) : width(width), height(height), title(title) {
  glfwInit();
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

  window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
  glfwSetWindowUserPointer(window, this);
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

  firstMouse = true;
}

Window::~Window() {
  glfwDestroyWindow(window);
  glfwTerminate();
}

void Window::createSurface(VkInstance instance, VkSurfaceKHR *surface) {
  if (glfwCreateWindowSurface(instance, window, nullptr, surface) != VK_SUCCESS) {
    throw std::runtime_error("failed to craete window surface");
  }
}

void Window::framebufferSizeCallback(GLFWwindow *window, int width, int height) {
  auto win = reinterpret_cast<Window *>(glfwGetWindowUserPointer(window));
  win->resized = true;
  win->width = width;
  win->height = height;
}
