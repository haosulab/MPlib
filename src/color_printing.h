#pragma once

#include <iostream>

const std::string red("\033[0;31m");
const std::string green("\033[0;32m");
const std::string yellow("\033[0;33m");
const std::string blue("\033[0;34m");
const std::string magenta("\033[0;35m");
const std::string cyan("\033[0;36m");
const std::string reset("\033[0m");

template<typename... Args>
void print_info(Args... args) {
  // use blue
  std::cout << blue;
  (std::cout << ... << args) << reset << std::endl;
}

template<typename... Args>
void print_error(Args... args) {
    // use red
    std::cerr << red;
    (std::cerr << ... << args) << reset << std::endl;
}

template<typename... Args>
void print_warning(Args... args) {
  // use yellow
  std::cout << yellow;
  (std::cout << ... << args) << reset << std::endl;
}

template<typename... Args>
void print_debug(Args... args) {
  // use cyan
  std::cout << cyan;
  (std::cout << ... << args) << reset << std::endl;
}

template<typename... Args>
void print_verbose(Args... args) {
  // use megenta
  std::cout << magenta;
  (std::cout << ... << args) << reset << std::endl;
}
