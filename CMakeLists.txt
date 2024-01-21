cmake_minimum_required(VERSION 3.12)
project(mp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g3 -Wall -Werror -fsized-deallocation -Wno-deprecated-declarations")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# set -fuse-ld=lld if lld is found
find_program(LLD_FOUND ld.lld)
if(LLD_FOUND)
    message(STATUS "Using lld")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=lld")
endif(LLD_FOUND)

# add ccache as compiler launcher
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    message(STATUS "Using ccache")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
endif(CCACHE_FOUND)

# Pinocchio uses its own FindCppAD, but does not provide it.
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(Boost_NO_WARN_NEW_VERSIONS 1)  # silence Boost CMake warnings
find_package(Eigen3 3.4.0 REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(pinocchio REQUIRED)
find_package(assimp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS})
include_directories("src")

# store libries in a variable
set(LIBS ompl fcl assimp orocos-kdl Boost::system Boost::filesystem urdfdom_model urdfdom_world)

# pymp
file(GLOB_RECURSE PROJECT_SRC "src/*.h" "src/*.cpp" "src/*.hpp")
add_library(mp STATIC ${PROJECT_SRC})
target_link_libraries(mp PRIVATE ${LIBS})
set_target_properties(mp PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

# pybind11_mkdoc
add_custom_target(
  pybind11_mkdoc ALL
  COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/dev/mkdoc.sh"
               "-I$<JOIN:$<TARGET_PROPERTY:mp,INCLUDE_DIRECTORIES>,;-I>"
  BYPRODUCTS "${CMAKE_CURRENT_SOURCE_DIR}/python/docstring/*.h"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/dev/mkdoc.sh" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND_EXPAND_LISTS
  VERBATIM
)

# Pybind11
add_subdirectory("third_party/pybind11")
include_directories("python")
pybind11_add_module(pymp python/pybind.cpp)
target_link_libraries(pymp PRIVATE mp)
add_dependencies(pymp pybind11_mkdoc)

# compile test_articulated_model and run the test
add_executable(test_articulated_model tests/test_articulated_model.cpp)
target_link_libraries(test_articulated_model PRIVATE mp)
add_test(NAME test_articulated_model COMMAND test_articulated_model)
