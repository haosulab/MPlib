FROM quay.io/pypa/manylinux2014_x86_64

RUN yum update && mkdir /workspace
WORKDIR /workspace

# basic development tools
RUN yum install -y eigen3-devel tinyxml-devel wget && yum clean all

# -------------------------------------------------------------------------- #
# OMPL
# -------------------------------------------------------------------------- #
# libccd
RUN git clone --single-branch -b v2.1 --depth 1 https://github.com/danfis/libccd.git && \
    cd libccd && mkdir build && cd build && \
    cmake -G "Unix Makefiles" .. && make -j && make install && \
    rm -rf /workspace/libccd

# boost (require >= 1.58)
# Reference: https://www.boost.org/doc/libs/1_76_0/more/getting_started/unix-variants.html#easy-build-and-install
# NOTE(jigu): there are compilation errors when boost.python is also built.
# To build boost.python, maybe we need to refer to https://www.boost.org/doc/libs/1_35_0/libs/python/doc/building.html#examples
RUN wget https://boostorg.jfrog.io/artifactory/main/release/1.76.0/source/boost_1_76_0.tar.gz && \
    tar -xf boost_1_76_0.tar.gz && \
    rm boost_1_76_0.tar.gz && \
    cd boost_1_76_0 && ./bootstrap.sh --without-libraries=python && ./b2 install && \
    rm -rf /workspace/boost_1_76_0

# OMPL
RUN git clone --single-branch -b 1.5.0 --depth 1 --recurse-submodules https://github.com/ompl/ompl.git && \
    cd ompl && mkdir build && cd build && \
    cmake .. && make -j && make install && \
    rm -rf /workspace/ompl

# -------------------------------------------------------------------------- #
# FCL
# -------------------------------------------------------------------------- #
# octomap (for octree collision)
RUN git clone --single-branch -b v1.9.7 --depth 1 https://github.com/OctoMap/octomap.git && \
    cd octomap/octomap && mkdir build && cd build && \
    cmake .. && make -j && make install && \
    rm -rf /workspace/octomap

# v0.6.1 does not work, use newer instead
RUN git clone --single-branch https://github.com/flexible-collision-library/fcl.git && \
    cd fcl && git checkout 7fcdc7f09bedb3d9544bfce067b01298873ad906 && mkdir build && cd build && \
    cmake .. && make -j && make install && \
    rm -rf /workspace/fcl

# -------------------------------------------------------------------------- #
# pinocchio
# -------------------------------------------------------------------------- #
RUN git clone --single-branch -b 20210000.6 --depth 1 https://github.com/coin-or/CppAD.git && \
    cd CppAD && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    rm -rf /workspace/CppAD

RUN git clone --single-branch -b 0.3.2 --depth 1 https://github.com/ros/console_bridge.git && \
    cd console_bridge && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    rm -rf /workspace/console_bridge

RUN git clone --single-branch -b 1.0.5 --depth 1 https://github.com/ros/urdfdom_headers.git && \
    cd urdfdom_headers && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    rm -rf /workspace/urdfdom_headers

RUN git clone --single-branch -b 1.0.4 --depth 1 https://github.com/ros/urdfdom.git && \
    cd urdfdom && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    rm -rf /workspace/urdfdom

RUN git clone --single-branch -b v2.5.6 --depth 1 https://github.com/stack-of-tasks/pinocchio.git && \
    cd pinocchio && git submodule update --init --recursive && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_AUTODIFF_SUPPORT=ON -DBUILD_WITH_URDF_SUPPORT=ON && make -j && make install && \
    rm -rf /workspace/pinocchio

# -------------------------------------------------------------------------- #
# Others
# -------------------------------------------------------------------------- #
RUN git clone --single-branch -b v5.0.1 --depth 1 https://github.com/assimp/assimp.git && \
    cd assimp && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DASSIMP_BUILD_TESTS=OFF && make -j && make install && \
    rm -rf /workspace/assimp

RUN git clone --single-branch -b v1.5.0 --depth 1 https://github.com/orocos/orocos_kinematics_dynamics.git && \
    cd orocos_kinematics_dynamics/orocos_kdl && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install && \
    rm -rf /workspace/orocos_kinematics_dynamics

RUN useradd -rm -d /home/user -s /bin/bash -g root -u 1000 user