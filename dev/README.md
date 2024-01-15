# Developer Notes

## Getting Started

We have provided a docker image [`kolinguo/mplib-build`](https://hub.docker.com/r/kolinguo/mplib-build) based on
`quay.io/pypa/manylinux2014_x86_64` base image, to compile and package the library.
Most dependencies have been installed.
The docker file can be found [here](../docker/Dockerfile).

To building python wheels, run `./dev/build_wheels.sh --py 310`  
This will install [`cibuildwheel`](https://cibuildwheel.readthedocs.io/en/stable/#how-it-works) via pip and use it to build wheel for the specified python version.  
The wheel will be generated in the `wheelhouse/`

If you want to start a docker container for debugging, run `./dev/docker_setup.sh`

### Versioning & Release Process
Currently, the project is setup to use [`setuptools-git-versioning`](https://setuptools-git-versioning.readthedocs.io/en/v1.13.5/schemas/tag/tag_release.html)
which automatically sets the python package version based on the latest tag.
This means that there's no need to manually update the package version in `pyproject.toml` or `setup.py`.

If there are no tags in the current branch, the package version will be determined based on the
[VERSION](VERSION) file.

#### Release Process
* Tag commit in the `main` branch with the next release version (e.g., `v0.1.0`).
  * When the tag is pushed, it triggers a GitHub action to build wheels
  for all supported python versions and publish to PyPI.
* Save the next release version (e.g., `v0.2.0`) in the [VERSION](VERSION) file.

Future commits will trigger GitHub action to build nightly wheels with version `v0.2.0.dev{timestamp}+git.{sha}`.

Please do not tag commits in any other branch other than `main`!

### Code Editor Setup

For Visual Studio Code, it is suggested to install extensions relevant to: C/C++, CMake.  
You could add the following entries in the setting for convenience:

- `"compileCommands": "${workspaceFolder}/build/compile_commands.json"`:
parse include directories from cmake results

For other editors (*e.g.*, vim), use [`clangd`](https://clangd.llvm.org/design/compile-commands).  
The `compile_commands.json` file can be generated with
`CMAKE_EXPORT_COMPILE_COMMANDS=ON cmake ..`

## Develop locally (use without docker)

### Dependencies

1. Linear algebra library `Eigen3`, which one can find [here](https://eigen.tuxfamily.org/index.php?title=Main_Page) and install by following the instructions in the unzipped file called `INSTALL`. If one is on Ubuntu, one can also install it by `sudo apt install libeigen3-dev`.
2. C++ extras library `Boost` by following the instructions [here](https://www.boost.org/doc/libs/1_76_0/more/getting_started/unix-variants.html). Similarly, Ubuntu users get a shortcut by `sudo apt install libboost-all-dev`.
3. Collision detection `libccd`. Follow the guides [here](https://github.com/danfis/libccd#compile-and-install) or do `sudo apt install libccd-dev`.
4. Open Motional Planning Library `ompl`. Install by following these [instructions](https://ompl.kavrakilab.org/installation.html). Ubuntu users can also do `sudo apt install libompl-dev`.
5. Point cloud library `octomap`. Install from source [here](https://github.com/OctoMap/octomap.git). Ubuntu users can also do `sudo apt install liboctomap-dev`.
6. Flexible Collision Library `fcl`. This depends on `Eigen3` and `libccd`. Follow the instructions [here](https://github.com/flexible-collision-library/fcl/blob/master/INSTALL). Ubuntu users can also do `sudo apt install libfcl-dev`. (Note: you can also add in the faster hpp-fcl from [here](https://github.com/humanoid-path-planner/hpp-fcl/blob/devel/INSTALL))
7. Asset import library `assimp`. Install instructions [here](https://github.com/assimp/assimp/blob/master/Build.md)
8. URDF parser `urdfdom`. Clone from [this repo](https://github.com/ros/urdfdom) and do the usual cmake+make+make install. Ubuntu users can also do `sudo apt install liburdfdom-dev`.
9. Rigid-body dynamics library `pinocchio`. Full installation instructions [here](https://stack-of-tasks.github.io/pinocchio/download.html). It's recommended if you install from source as other methods might not configure the paths well.
10. Additional dynamics library `orocos_kdl`. Install by following the instructions [here](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/INSTALL.md). Ubuntu users can also do `sudo apt install liborocos-kdl-dev`.

### Compile

One can compile the dynamic library by doing the following:

1. `mkdir build && cd build`
2. `cmake .. && make -j8`

Depending on your python version, you will get a file called `pymp.cpython-310-x86_64-linux-gnu.so` or similar. This is directly importable in python by doing `import pymp`.

To install the entire package along with python glue code, do `python3.[version] -m pip install .` inside the root directory of the project.

## Stubs & Documentation Generation

To generate stubs and documentations, run `./dev/generate_stub_and_doc.sh`.
By default it uses `python3.10` in docker image [`kolinguo/mplib-build`](https://hub.docker.com/r/kolinguo/mplib-build).

The script does the following:
* Build a python wheel using [`cibuildwheel`](https://cibuildwheel.readthedocs.io/en/stable/#how-it-works).
* In a docker container, install the python wheel and
use [`pybind11-stubgen`](https://github.com/sizmailov/pybind11-stubgen)
to generate stubs.  
Copy the generated stubs into [`mplib`](../mplib/).
* In a docker container, install the python wheel and
use [`pdoc`](https://pdoc.dev/docs/pdoc.html) to generate documentations.  
Copy the generated docs into [`docs`](../docs/).

## GitHub Action CI/CD
Currently, a GitHub action is setup to build / release / publish python wheels.

### Push to `main` branch
This triggers building wheels for all supported python versions and
creating a [nightly release](https://github.com/haosulab/MPlib/releases/tag/nightly)
with the built wheels.

### Push tag to branch
This triggers building wheels for all supported python versions and
publishing them to [MPlib PyPI](https://pypi.org/p/mplib/).

## FAQ

<details>
<summary>ImportError: dynamic module does not define module export function</summary>

Please check whether your extension file `*.so` has the same name as `PYBIND11_MODULE(*, m)`

</details>
