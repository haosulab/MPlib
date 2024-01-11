FROM quay.io/pypa/manylinux2014_x86_64

# basic development tools
RUN yum update -y && yum install -y wget git && yum upgrade -y && yum clean all
WORKDIR /workspace

# Eigen
RUN git clone --single-branch -b 3.4.0 --depth 1 https://gitlab.com/libeigen/eigen.git && \
    cd eigen && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc) && make install && \
    rm -rf /workspace/eigen

# -------------------------------------------------------------------------- #
# OMPL
# -------------------------------------------------------------------------- #
# libccd
RUN git clone --single-branch -b v2.1 --depth 1 https://github.com/danfis/libccd.git && \
    cd libccd && mkdir build && cd build && \
    cmake -G "Unix Makefiles" .. && make -j$(nproc) && make install && \
    rm -rf /workspace/libccd

# boost (require >= 1.58)
# Reference: https://www.boost.org/doc/libs/1_76_0/more/getting_started/unix-variants.html#easy-build-and-install
# NOTE(jigu): there are compilation errors when boost.python is also built.
# To build boost.python, maybe we need to refer to https://www.boost.org/doc/libs/1_35_0/libs/python/doc/building.html#examples
RUN wget https://boostorg.jfrog.io/artifactory/main/release/1.84.0/source/boost_1_84_0.tar.gz && \
    tar -xf boost_1_84_0.tar.gz && \
    rm boost_1_84_0.tar.gz && \
    cd boost_1_84_0 && ./bootstrap.sh --without-libraries=python && ./b2 install && \
    rm -rf /workspace/boost_1_84_0

# OMPL
RUN git clone --single-branch -b 1.6.0 --depth 1 --recurse-submodules https://github.com/ompl/ompl.git && \
    cd ompl && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    rm -rf /workspace/ompl

# -------------------------------------------------------------------------- #
# FCL
# -------------------------------------------------------------------------- #
# octomap (for octree collision)
RUN git clone --single-branch -b v1.9.8 --depth 1 https://github.com/OctoMap/octomap.git && \
    cd octomap/octomap && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    rm -rf /workspace/octomap

# v0.6.1 does not work, use newer instead
RUN git clone --single-branch -b 0.7.0 --depth 1 https://github.com/flexible-collision-library/fcl.git && \
    cd fcl && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    rm -rf /workspace/fcl

# -------------------------------------------------------------------------- #
# pinocchio
# -------------------------------------------------------------------------- #
RUN git clone --single-branch -b 20240000.1 --depth 1 https://github.com/coin-or/CppAD.git && \
    cd CppAD && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install && \
    rm -rf /workspace/CppAD

RUN git clone --single-branch -b 1.0.2 --depth 1 https://github.com/ros/console_bridge.git && \
    cd console_bridge && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install && \
    rm -rf /workspace/console_bridge

RUN git clone --single-branch -b 1.1.1 --depth 1 https://github.com/ros/urdfdom_headers.git && \
    cd urdfdom_headers && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install && \
    rm -rf /workspace/urdfdom_headers

RUN git clone --single-branch -b 10.0.0 --depth 1 https://github.com/leethomason/tinyxml2.git && \
    cd tinyxml2 && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-fPIC" && make -j$(nproc) && make install && \
    rm -rf /workspace/tinyxml2

RUN git clone --single-branch -b 4.0.0 --depth 1 https://github.com/ros/urdfdom.git && \
    cd urdfdom && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install && \
    rm -rf /workspace/urdfdom

RUN git clone --single-branch -b v2.6.21 --depth 1 https://github.com/stack-of-tasks/pinocchio.git && \
    cd pinocchio && git submodule update --init --recursive && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_PYTHON_INTERFACE=OFF \
      -DBUILD_WITH_AUTODIFF_SUPPORT=ON -DBUILD_WITH_URDF_SUPPORT=ON && \
    make -j$(nproc) && make install && \
    rm -rf /workspace/pinocchio

# -------------------------------------------------------------------------- #
# Others
# -------------------------------------------------------------------------- #
RUN git clone --single-branch -b v5.3.1 --depth 1 https://github.com/assimp/assimp.git && \
    cd assimp && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DASSIMP_BUILD_TESTS=OFF && make -j$(nproc) && make install && \
    rm -rf /workspace/assimp

RUN git clone --single-branch -b v1.5.1 --depth 1 https://github.com/orocos/orocos_kinematics_dynamics.git && \
    cd orocos_kinematics_dynamics/orocos_kdl && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install && \
    rm -rf /workspace/orocos_kinematics_dynamics

# Set working directory to be repository directory
ARG WORKDIR="/workspace"
WORKDIR ${WORKDIR}
