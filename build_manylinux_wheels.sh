#!/usr/bin/env bash
set -euxo pipefail

# Build manylinux wheels for multiple CPython versions in one container.

# --- configurable targets ---
PY_TAGS=(
  cp39-cp39
  cp310-cp310
  cp311-cp311
  cp312-cp312
)

PLAT=manylinux2014_x86_64
REPO_ROOT=/workspace
WHEELHOUSE="${REPO_ROOT}/wheelhouse"
REPAIRED="${REPO_ROOT}/repaired_wheels"

mkdir -p "${WHEELHOUSE}" "${REPAIRED}"
cd "${REPO_ROOT}"

# Speed up native builds
export CMAKE_BUILD_PARALLEL_LEVEL="$(nproc)"
export MAKEFLAGS="-j$(nproc)"

for TAG in "${PY_TAGS[@]}"; do
  PYBIN="/opt/python/${TAG}/bin"

  # Make this interpreter's bin first on PATH so scripts calling `python3` will resolve correctly
  export PATH="${PYBIN}:${PATH}"

  # Install per-interpreter build toolchain and build-time deps
  "${PYBIN}/pip" install -U pip setuptools wheel build auditwheel
  "${PYBIN}/pip" install "setuptools-git-versioning<2" "libclang==11.0.1"
  # If you need headers like pybind11 / numpy, uncomment as needed:
  # "${PYBIN}/pip" install pybind11 "numpy<2"

  # Clean cross-version artifacts to avoid ABI/linkage pollution
  rm -rf build/ *.egg-info dist/ || true

  # Build wheel for this Python interpreter (PEP 517)
  "${PYBIN}/python" -m build --wheel --no-isolation -C--build-option=--verbose

  # Move wheel(s) into a common wheelhouse
  mv dist/*.whl "${WHEELHOUSE}"
done

# Use a known auditwheel binary (cp312) to repair all wheels
AUDITWHEEL_BIN="/opt/python/cp312-cp312/bin/auditwheel"
if ! [ -x "${AUDITWHEEL_BIN}" ]; then
  # Fallback to whatever is on PATH if cp312 auditwheel is unavailable
  AUDITWHEEL_BIN="$(command -v auditwheel)"
fi

# Repair wheels to bundle shared libs and stamp manylinux tag
shopt -s nullglob
for WHL in "${WHEELHOUSE}"/*.whl; do
  "${AUDITWHEEL_BIN}" repair --plat "${PLAT}" "${WHL}" -w "${REPAIRED}"
done
shopt -u nullglob

echo "All repaired wheels are in: ${REPAIRED}"
ls -1 "${REPAIRED}" || true
