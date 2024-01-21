#!/bin/bash

set -eEu -o pipefail

# Move to the repo folder, so later commands can use relative paths
SCRIPT_PATH=$(readlink -f "$0")
SCRIPT_DIR=$(dirname "$SCRIPT_PATH")
cd "$SCRIPT_DIR"

rm -rfv docstring_drake.h.tmp_artifacts

python3 /MPlib/dev/test_mkdoc/drake/mkdoc.py \
  -output=docstring_drake.h \
  -I/opt/rh/llvm-toolset-11.0/root/usr/lib64/clang/11.0.1/include \
  -I. -I/MPlib/dev \
  -std=c++17 \
  ./sample_header.h
