#!/bin/bash

# Docker image name to build wheel
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
# Section 1: Build python wheel in a docker container      #
############################################################
# Move to the repo folder, so later commands can use relative paths
SCRIPT_PATH=$(readlink -f "$0")
REPO_DIR=$(dirname "$(dirname "$SCRIPT_PATH")")
cd "$REPO_DIR"

echo_info() {
  echo -e "\n\e[1;36m$1 ...\e[0m"
}

PY_VERSION=
while (("$#")); do
  case "$1" in
  --py | --python)
    if [ -n "$2" ] && [ ${2:0:1} != "-" ]; then
      PY_VERSION=$2
      shift 2
    else
      echo "Error: Argument for $1 is missing" >&2
      exit 1
    fi
    ;;
  *) # unsupported flags
    echo "Error: Unsupported flag $1" >&2
    exit 2
    ;;
  esac
done

# Actual function to build wheel
build_wheel() {
  BUILD_WHEEL_CMD="\
    export PATH=\"\$(find /opt/python -name \"cp${PY_VERSION}*\")/bin:\${PATH}\" \
    && rm -rf build dist/* wheelhouse/* \
    && git config --global --add safe.directory '*' \
    && python3 -m build --wheel \
    && auditwheel repair \$(find dist/ -name \"*.whl\")
  "

  echo_info "Building wheel for python${PY_VERSION} in docker '${IMGNAME}'"
  local temp_cont_name="mplib_build_$(date "+%Y%m%d_%H%M%S")"
  docker create --name="$temp_cont_name" \
    "$IMGNAME" \
    bash -c "$BUILD_WHEEL_CMD"
  docker cp . "${temp_cont_name}:/MPlib"
  docker start -a "$temp_cont_name"
  docker cp "${temp_cont_name}:/MPlib/wheelhouse" .
  docker cp "${temp_cont_name}:/MPlib/pybind/docstring" ./pybind
  docker rm -f "$temp_cont_name"
}

if [ -z "$PY_VERSION" ]; then
  echo "Error: No python version is provided"
  exit 3
fi

if [ "$PY_VERSION" == "all" ]; then
  # python3 -m cibuildwheel --platform linux
  for PY_VERSION in 38 39 310 311 312; do
    build_wheel
  done
else
  # CIBW_BUILD="cp${PY_VERSION}-*" python3 -m cibuildwheel --platform linux
  case "$PY_VERSION" in
    38|39|310|311|312) ;;
    *)
      echo "Error: Python version($PY_VERSION) not supported" >&2
      exit 4
      ;;
  esac

  build_wheel
fi
