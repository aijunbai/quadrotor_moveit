#ifndef UTILS_H
#define UTILS_H

#include <iostream>

#define PRINT_VALUE(x) \
  do { \
  std::cerr << #x " = '" << x << "'" << std::endl; \
  } while(0)

#endif // UTILS_H
