#!/usr/bin/env bash

PY36_BIN=/opt/python/cp36-cp36m/bin
PY36_INCLUDE=/opt/python/cp36-cp36m/include/python3.6m
PY38_BIN=/opt/python/cp38-cp38/bin
PY38_INCLUDE=/opt/python/cp38-cp38/include/python3.8

PY_BIN=$PY36_BIN
PY_INCLUDE=$PY36_INCLUDE

echo "export PATH=${PY_BIN}:\$PATH" >> ~/.bashrc
echo "export CPLUS_INCLUDE_PATH=${PY_INCLUDE}:\$CPLUS_INCLUDE_PATH" >> ~/.bashrc