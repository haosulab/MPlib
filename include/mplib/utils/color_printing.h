#pragma once

#include <iostream>
#include <string_view>

namespace mplib {

inline constexpr std::string_view red {"\033[0;31m"};
inline constexpr std::string_view green {"\033[0;32m"};
inline constexpr std::string_view yellow {"\033[0;33m"};
inline constexpr std::string_view blue {"\033[0;34m"};
inline constexpr std::string_view magenta {"\033[0;35m"};
inline constexpr std::string_view cyan {"\033[0;36m"};
inline constexpr std::string_view reset {"\033[0m"};

template <typename... Args>
void print_debug(const Args &...args) {
  // use cyan
  std::cout << cyan;
  (std::cout << ... << args) << reset << std::endl;
}

template <typename... Args>
void print_verbose(const Args &...args) {
  // use megenta
  std::cout << magenta;
  (std::cout << ... << args) << reset << std::endl;
}

template <typename... Args>
void print_info(const Args &...args) {
  // use blue
  std::cout << blue;
  (std::cout << ... << args) << reset << std::endl;
}

template <typename... Args>
void print_warning(const Args &...args) {
  // use yellow
  std::cout << yellow;
  (std::cout << ... << args) << reset << std::endl;
}

template <typename... Args>
void print_error(const Args &...args) {
  // use red
  std::cerr << red;
  (std::cerr << ... << args) << reset << std::endl;
}

}  // namespace mplib
