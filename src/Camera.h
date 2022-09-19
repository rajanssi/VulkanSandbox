#pragma once

#define GLFW_INCLUDE_VULKAN
#include "GLFW/glfw3.h"
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
public:
  // TODO: This definitely shuldn't be here -- Needs some kind of input class
  struct {
    int right = GLFW_KEY_RIGHT;
    int left = GLFW_KEY_LEFT;
    int up = GLFW_KEY_UP;
    int down = GLFW_KEY_DOWN;
    int W = GLFW_KEY_W;
    int A = GLFW_KEY_A;
    int S = GLFW_KEY_S;
    int D = GLFW_KEY_D;
    int Q = GLFW_KEY_D;
    int E = GLFW_KEY_D;
  } keys;

  const glm::vec3 WorldUp = glm::vec3(0.0f, 1.0f, 0.0f);
  glm::vec3 position = glm::vec3(0.0f, 0.0f, 2.0f);
  glm::vec3 front;
  glm::vec3 up;
  glm::vec3 right;
  // euler Angles
  float yaw = -90.0f;
  float pitch = 0.0f;
  // camera options
  float movementSpeed = 0.5f;
  float mouseSensitivity = 0.08f;


  Camera() { updateCameraVectors(); }
  void setPerspectiveProjection(float fovY, float aspect, float near, float far);
  void update(float deltaTime, GLFWwindow *window);
  void processMouseMovement(float xoffset, float yoffset);

  const glm::mat4 &getProjection() const { return perspective; };
  const glm::mat4 getViewMatrix() const { return glm::lookAt(position, position + front, up); }
  void updateCameraVectors();
private:
  glm::mat4 perspective{1.f};
};
