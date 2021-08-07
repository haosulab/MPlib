#!/usr/bin/env bash

PY_VERSION=
PACKAGE_VERSION=
REPAIR_ONLY=

while (( "$#" )); do
  case "$1" in
    --py|--python)
        if [ -n "$2" ] && [ ${2:0:1} != "-" ]; then
            PY_VERSION=$2
            shift 2
        else
            echo "Error: Argument for $1 is missing" >&2
            exit 1
        fi
        ;;
    -v|--version)
        if [ -n "$2" ] && [ ${2:0:1} != "-" ]; then
            PACKAGE_VERSION=$2
            shift 2
        else
            echo "Error: Argument for $1 is missing" >&2
            exit 1
        fi
        ;;
    --repair-only)
        REPAIR_ONLY=1
        shift
        ;;
    *) # unsupported flags
        echo "Error: Unsupported flag $1" >&2
        exit 1
        ;;
  esac
done

function build_wheel() {
    if (( $PY_VERSION == 36 )); then
        PY_DOT=3.6
        EXT="m"
    elif (( $PY_VERSION == 37 )); then
        PY_DOT=3.7
        EXT="m"
    elif (( $PY_VERSION == 38 )); then
        PY_DOT=3.8
        EXT=""
    elif (( $PY_VERSION == 39 )); then
        PY_DOT=3.9
        EXT=""
    else
        echo "Error: Python version($PY_VERSION) not found"
        exit 2
    fi

    if [ -z "$REPAIR_ONLY" ]; then
        # if [ -d build/lib.linux-x86_64-${PY_DOT} ]; then
        #     echo "Delete old build/lib.linux-x86_64-${PY_DOT}"
        #     rm -rf "build/lib.linux-x86_64-${PY_DOT}"
        # fi

        INCLUDE_PATH=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/include/python${PY_DOT}${EXT}
        BIN=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/bin/python
        echo "Using bin path ${BIN}"
        echo "Using include path ${INCLUDE_PATH}"

        export CPLUS_INCLUDE_PATH=$INCLUDE_PATH
        COMMAND="${BIN} setup.py bdist_wheel"
        echo "Running command ${COMMAND}"
        eval "$COMMAND"
    fi

    if [ -n "$PACKAGE_VERSION" ]; then
        echo "mplib verion ${PACKAGE_VERSION}"
        WHEEL_FILE="./dist/mplib-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}${EXT}-linux_x86_64.whl"
        if [ -f "$WHEEL_FILE" ]; then
            echo "$WHEEL_FILE exist, begin audit and repair"
        fi
        WHEEL_COMMAND="auditwheel repair ${WHEEL_FILE}"
        eval "$WHEEL_COMMAND"
    fi
}

if [ -z "$PY_VERSION" ]; then
    echo "Error: No python version is provided"
    exit 3
fi

if [ "$PY_VERSION" == "all" ]; then
    for PY_VERSION in 36 37 38 39; do
        build_wheel
    done
else
    build_wheel
fi