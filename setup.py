# -*- coding: utf-8 -*-
import os
import sys
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        #extdir = "/home/pip_build/pymp"
        print(extdir)
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cfg = "Debug" if self.debug else "Release"

        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        # Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={}".format(extdir),
            "-DPYTHON_EXECUTABLE={}".format(sys.executable),
            "-DCMAKE_BUILD_TYPE={}".format(cfg),  # not used on MSVC, but no harm
        ]
        build_args = []

        #if not cmake_generator:
        #    cmake_args += ["-GNinja"]

        self.parallel = 16
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            if hasattr(self, "parallel") and self.parallel:
                build_args += ["-j{}".format(self.parallel)]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        print(cmake_args, build_args)

        subprocess.check_call(
            ["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp
        )
        subprocess.check_call(
            ["cmake", "--build", "."] + build_args, cwd=self.build_temp
        )

setup(
    name="pymp",
    version="0.0.1",
    author_email="minghua@ucsd.edu",
    description="A simple motion planning library",
    ext_modules=[CMakeExtension("_pymp")],
    install_requires=["numpy >= 1.17", "toppra >= 0.4.0", "transforms3d >= 0.3.1"],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
    packages=["pymp"],
    package_dir = {"pymp": "pymp/"}
)
