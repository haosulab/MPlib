#!/bin/bash

# Python version to build wheels and generate stubs
PY_VERSION=310
# Docker image name to install wheel and generate stubs
IMGNAME="kolinguo/mplib-build:latest"

############################################################
# Section 0: Bash Error Handling                           #
############################################################
set -eEu -o pipefail
trap 'catch' ERR  # Trap all errors (status != 0) and call catch()
catch() {
  local err="$?"
  local err_command="$BASH_COMMAND"
  set +xv  # disable trace printing

  echo -e "\n\e[1;31mCaught error in ${BASH_SOURCE[1]}:${BASH_LINENO[0]} ('${err_command}' exited with status ${err})\e[0m" >&2
  echo "Traceback (most recent call last, command might not be complete):" >&2
  for ((i = 0; i < ${#FUNCNAME[@]} - 1; i++)); do
    local funcname="${FUNCNAME[$i]}"
    [ "$i" -eq "0" ] && funcname=$err_command
    echo -e "  ($i) ${BASH_SOURCE[$i+1]}:${BASH_LINENO[$i]}\t'${funcname}'" >&2
  done
  exit "$err"
}

############################################################
# Section 1: Build python wheels                           #
############################################################
# Move to the repo folder, so later commands can use relative paths
SCRIPT_PATH=$(readlink -f "$0")
REPO_DIR=$(dirname "$(dirname "$SCRIPT_PATH")")
cd "$REPO_DIR"
REPO_NAME=$(basename "$REPO_DIR")  # default WORKDIR inside container

echo_info() {
  echo -e "\n\e[1;36m$1 ...\e[0m"
}

echo_info "Removing previous wheels under 'wheelhouse/'"
rm -rfv wheelhouse/*.whl

echo_info "Building wheels with 'dev/build_wheels.sh'"
dev/build_wheels.sh --py "$PY_VERSION"

############################################################
# Section 2: Build stubs                                   #
############################################################
# Build stub and run ruff isort / formatter
BUILD_STUB_CMD="\
  export PATH=\"\$(find /opt/python -name \"cp${PY_VERSION}*\")/bin:\${PATH}\" \
  && python3 -m pip install pybind11-stubgen \
  && python3 -m pip install wheelhouse/mplib*.whl \
  && python3 dev/stubgen.py \
  && python3 -m pip install ruff \
  && ruff check --no-cache --select I --fix ./stubs \
  && ruff format --no-cache ./stubs \
  && chown -R \${USER_UID}:\${USER_UID} ./stubs
"

echo_info "Building stubs in docker '${IMGNAME}'"
docker run -it --rm \
  -e USER_UID="$UID" \
  -v "$REPO_DIR":/${REPO_NAME} \
  "$IMGNAME" \
  bash -c "$BUILD_STUB_CMD"

echo_info "Removing previous stubs under 'mplib/'"
find mplib -name "*.pyi" -exec rm -v {} \;
echo_info "Moving generated stubs into 'mplib/pymp'"
mv -v stubs/mplib/pymp/* mplib/pymp
echo_info "Removing 'stubs/'"
rm -rfv stubs/
