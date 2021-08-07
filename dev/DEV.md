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
