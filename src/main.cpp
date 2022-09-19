#include <iostream>

#include "Application.h"

int main() {
  Application app;
  try {
    app.run();
  } catch (std::exception e) {
    std::cerr << e.what() << '\n';
  }

  return 0;
}
