#!/bin/bash

set -eEu -o pipefail

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

if [ -z "$PY_VERSION" ]; then
  echo "Error: No python version is provided"
  exit 3
fi

if ! command -v "cibuildwheel" &>/dev/null; then
  python3 -m pip install cibuildwheel
fi

if [ "$PY_VERSION" == "all" ]; then
  cibuildwheel --platform linux
else
  CIBW_BUILD="cp${PY_VERSION}-*" cibuildwheel --platform linux
fi
