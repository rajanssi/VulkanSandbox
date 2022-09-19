#include "Camera.h"

#include <cassert>
#include <limits>

void Camera::setPerspectiveProjection(float fovY, float aspect, float near, float far) {
  assert(glm::abs(aspect - std::numeric_limits<float>::epsilon()) > 0.0f);
  perspective = glm::perspective(glm::radians(fovY), aspect, near, far);
  perspective[1][1] *= -1.0f;
}

void Camera::update(float deltaTime, GLFWwindow *window) {
  float velocity = movementSpeed * deltaTime;
  if (glfwGetKey(window, keys.W) == GLFW_PRESS)
    position += front * velocity;
  if (glfwGetKey(window, keys.S) == GLFW_PRESS)
    position -= front * velocity;
  if (glfwGetKey(window, keys.A) == GLFW_PRESS)
    position -= right * velocity;
  if (glfwGetKey(window, keys.D) == GLFW_PRESS)
    position += right * velocity;
}

void Camera::processMouseMovement(float xoffset, float yoffset) {
  xoffset *= mouseSensitivity;
  yoffset *= mouseSensitivity;

  yaw += xoffset;
  pitch += yoffset;

  // NOTE: Pitch is constrained
  if (pitch > 89.0f)
    pitch = 89.0f;
  if (pitch < -89.0f)
    pitch = -89.0f;

  updateCameraVectors();
}

void Camera::updateCameraVectors() {
  // calculate the new Front vector
  glm::vec3 front;
  front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
  front.y = sin(glm::radians(pitch));
  front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
  this->front = glm::normalize(front);
  // also re-calculate the Right and Up vector
  right = glm::normalize(glm::cross(this->front, WorldUp));
  up = glm::normalize(glm::cross(right, this->front));
}
