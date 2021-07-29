cmake_minimum_required(VERSION 3.10)
project(mp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE " -O3 -g3 -Wall -fsized-deallocation -Wno-deprecated-declarations")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(fcl REQUIRED)
find_package(ompl REQUIRED)
find_package(ccd REQUIRED)
find_package(assimp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)

include_directories("/usr/include/eigen3")
include_directories("/usr/local/include/ompl-1.5")

set(PINOCCHIO_LIBRARY ${pinocchio_LIBRARIES} ${Boost_LIBRARIES} urdfdom_model)
add_definitions(-DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR)
add_definitions(-DPINOCCHIO_WITH_URDFDOM)

include_directories(src)
file(GLOB_RECURSE PROJECT_SRC "src/*.h" "src/*.cpp" "src/*.hpp")

add_library(mp STATIC ${PROJECT_SRC})

target_link_libraries(mp PRIVATE ${PINOCCHIO_LIBRARY} fcl assimp orocos-kdl ompl ${OMPL_LIBRARIES})
set_target_properties(mp PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

pybind11_add_module(_mplib python/pybind.cpp)

target_link_libraries(_mplib PRIVATE mp)

