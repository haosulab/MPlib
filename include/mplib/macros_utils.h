#pragma once

#define ASSERT(exp, info)             \
  if (!(exp)) {                       \
    throw std::runtime_error((info)); \
  }
