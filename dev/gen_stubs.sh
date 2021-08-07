#!/usr/bin/env bash
# pip install pybind11-stubgen

if [ -d stubs ]; then
    rm -rf stubs
fi

pybind11-stubgen mplib --no-setup-py --ignore-invalid all