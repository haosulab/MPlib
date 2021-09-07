#!/usr/bin/env bash
# pip install pybind11-stubgen

if [ -d stubs ]; then
    rm -rf stubs
fi

python dev/stubgen.py mplib --no-setup-py --ignore-invalid all
rm -rf mplib/pymp &&  cp -r stubs/mplib-stubs/pymp mplib/pymp