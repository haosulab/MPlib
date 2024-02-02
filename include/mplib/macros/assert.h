#pragma once

#include <stdexcept>

#define ASSERT(exp, info)             \
  if (!(exp)) {                       \
    throw std::runtime_error((info)); \
  }
