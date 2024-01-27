#!/bin/bash
# Bash script to call mkdoc.py on all header files, used in CMakeLists.txt
# Install libclang for mkdoc.py
#   please match libclang version with /usr/lib/llvm-{ver}/lib/clang/{clang-ver}/
# python3 -m pip install libclang=={clang-ver}

# Additional flags that clang understands can be passed in as well
CLANG_FLAGS="-std=c++17 ${@}"
PY_SCRIPT_PATH="dev/mkdoc.py"
CPP_INCLUDE_DIR="include/mplib"
OUTPUT_DOCSTRING_DIR="pybind/docstring"

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
# Section 1: Build docstring from C++ to use in pybind11   #
############################################################
# Move to the repo folder, so later commands can use relative paths
SCRIPT_PATH=$(readlink -f "$0")
REPO_DIR=$(dirname "$(dirname "$SCRIPT_PATH")")
cd "$REPO_DIR"

for filepath in `find "$CPP_INCLUDE_DIR" -name "*.h" ! -name "types.h" ! -path "*macros*"`; do
  output_path="${OUTPUT_DOCSTRING_DIR}/$(realpath --relative-to="$CPP_INCLUDE_DIR" "$filepath")"

  # Create output dir
  mkdir -p "$(dirname "$output_path")"

  python3 "$PY_SCRIPT_PATH" -o="$output_path" $CLANG_FLAGS "$filepath" &
done

# Wait for all background process to finish
wait $(jobs -rp)
