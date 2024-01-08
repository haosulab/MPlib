# Developer Notes

## Getting Started

We have provided a docker image `haosulab/mplib-build` based on pypa base image, to compile and package the library.
Most dependencies have been installed.
The docker file can be found [here](../docker/Dockerfile).

A typical workflow for development can be as follows:
- Create a docker container: `bash dev/start_docker.sh`
- Attach the VSCode to the container
- Build a wheel in the container: `bash dev/build_wheels.sh --py 36`. The repaired wheel will be generated in the `dist/` by default. If you want to generate a repaired wheel, please provide the current version to the script, e.g., `bash dev/build_wheels.sh --py 36 --version 0.0.4`. The repaired wheel will be generated in the `wheelhouse/` by default.
- The repaired wheel can be installed in the host.

## VSCode

It is suggested to install extensions relevant to: C/C++, CMake.

You could add the following entries in the setting for convenience:
- `"compileCommands": "${workspaceFolder}/build/compile_commands.json"`: parse include directories from cmake results

## FAQ

<details>
<summary>ImportError: dynamic module does not define module export function</summary>

Please check whether your extension file `*.so` has the same name as `PYBIND11_MODULE(*, m)`

</details>

## Develop locally (use without docker and python)

### Dependencies

1. Linear algebra library `Eigen3`, which one can find [here](https://eigen.tuxfamily.org/index.php?title=Main_Page) and install by following the instructions in the unzipped file called `INSTALL`. If one is on Ubuntu, one can also install it by `sudo apt install libeigen3-dev`.
2. C++ extras library `Boost` by following the instructions [here](https://www.boost.org/doc/libs/1_76_0/more/getting_started/unix-variants.html). Similarly, Ubuntu users get a shortcut by `sudo apt install libboost-all-dev`.
3. Collision detection `libccd`. Follow the guides [here](https://github.com/danfis/libccd#compile-and-install) or do `sudo apt install libccd-dev`.
4. Open Motional Planning Library `ompl`. Install by following these [instructions](https://ompl.kavrakilab.org/installation.html). Ubuntu users can also do `sudo apt install libompl-dev`.
5. Point cloud library `octomap`. Install from source [here](https://github.com/OctoMap/octomap.git). Ubuntu users can also do `sudo apt install liboctomap-dev`.
6. Flexible Collision Library `fcl`. This depends on `Eigen3` and `libccd`. Follow the instructions [here](https://github.com/flexible-collision-library/fcl/blob/master/INSTALL). Ubuntu users can also do `sudo apt install libfcl-dev`. (Note: you can also add in the faster hpp-fcl from [here](https://github.com/humanoid-path-planner/hpp-fcl/blob/devel/INSTALL))
7. Asset import library `assimp`. Install instructions [here](https://github.com/assimp/assimp/blob/master/Build.md)
8. URDF parser `urdfdom`. Clone from [this repo](https://github.com/ros/urdfdom) and do the usual cmake+make+make install. Ubuntu users can also do `sudo apt install liburdfdom-dev`.
9.  Rigid-body dynamics library `pinocchio`. Full installation instructions [here](https://stack-of-tasks.github.io/pinocchio/download.html). It's recommended if you install from source as other methods might not configure the paths well.
10. Additional dynamics library `orocos_kdl`. Install by following the instructions [here](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/INSTALL.md). Ubuntu users can also do `sudo apt install liborocos-kdl-dev`.

### Compile

One can compile the dynamic library by doing the following:

1. `mkdir build && cd build`
2. cmake .. && make -j8

Depending on your python version, you will get a file called `pymp.cpython-310-x86_64-linux-gnu.so` or similar. This is directly importable in python by doing `import pymp`.

To install the entire package along with python glue code, do `python3.[version] -m pip install .` inside the root directory of the project.

## Documentation Generation

### Stub Generation

Stubs are useful for type checking and IDE autocompletion. To generate stubs, you **first need to have mplib compiled and installed**. Then, do `python3.[version] -m pip install pybind11_stubgen` and then run `bash dev/generate_stubs.sh`. This will generate stubs for the entire project in the `stubs/` directory. Note that you might need to change the version of the python inside the script. The default is 3.11.

### Documentation Generation

We use `pdoc` to generate the documentation. First install a version of mplib referring to the section above.

Then, you will need to install `pdoc` with `python3.[version] -m pip install pdoc`. The reason the version is important is because `pdoc` will actually do analysis on the files, so it will scrape through the mplib installed in the site package. Then, run `bash dev/generate_docs.sh`. This will generate the documentation in the `docs/` directory. Note that you might need to change the version of the python inside the script. The default is 3.11.
