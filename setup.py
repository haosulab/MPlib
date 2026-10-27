import os
import shutil
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, find_namespace_packages, setup
from setuptools.command.build_ext import build_ext


# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "") -> None:
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(Path(sourcedir).resolve())


class CMakeBuild(build_ext):
    def build_extension(self, ext: CMakeExtension) -> None:
        # Must be in this form due to bug in .resolve() only fixed in Python 3.10+
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()

        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "Release"

        # CMake lets you override the generator - we need to check this.
        # Can be set with Conda-Build, for example.
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        # Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",  # not used on MSVC, but no harm
        ]

        # Ensure active Python/conda environment prefix is in CMAKE_PREFIX_PATH
        # so CMake finds pybind11, Pinocchio, OMPL, FCL, etc.
        prefix_paths = [sys.prefix]
        if "CONDA_PREFIX" in os.environ and os.environ["CONDA_PREFIX"] not in prefix_paths:
            prefix_paths.append(os.environ["CONDA_PREFIX"])
        if "CMAKE_PREFIX_PATH" in os.environ:
            for p in os.environ["CMAKE_PREFIX_PATH"].split(os.pathsep):
                if p and p not in prefix_paths:
                    prefix_paths.append(p)
        cmake_args.append(f"-DCMAKE_PREFIX_PATH={';'.join(prefix_paths)}")

        build_args = []
        # Adding CMake arguments set as environment variable
        # (needed e.g. to build for ARM OSx on conda-forge)
        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        if not cmake_generator and shutil.which("ninja"):
            cmake_args += ["-GNinja"]

        # Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
        # across all generators.
        # self.parallel is a Python 3 only way to set parallel jobs by hand
        # using -j in the build_ext call, not supported by pip or PyPA-build.
        self.parallel = 16
        if (
            "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ
            and hasattr(self, "parallel")
            and self.parallel
        ):
            # CMake 3.12+ only.
            build_args += [f"-j{self.parallel}"]

        build_temp = Path(self.build_temp) / ext.name
        if not build_temp.exists():
            build_temp.mkdir(parents=True)

        print(cmake_args, build_args)

        subprocess.run(
            ["cmake", ext.sourcedir, *cmake_args], cwd=build_temp, check=True
        )
        subprocess.run(
            ["cmake", "--build", ".", *build_args], cwd=build_temp, check=True
        )

        if getattr(self, "editable_mode", False) or self.inplace:
            pkg_dir = Path(ext.sourcedir) / "mplib"
            for file in extdir.glob("pymp*"):
                if file.is_file():
                    dest = pkg_dir / file.name
                    if file.resolve() != dest.resolve():
                        shutil.copy2(file, dest)


setup(
    name="mplib",
    packages=find_namespace_packages(include=["mplib*"]),
    package_data={"": ["**/*.pyi"], "mplib": ["py.typed"]},
    ext_modules=[CMakeExtension("mplib.pymp")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
)
