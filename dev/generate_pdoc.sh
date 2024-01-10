#!/usr/bin/env bash
# python3.[version] -m pip install pdoc
# run from the root of the project

current_dir=$(pwd)
cd ~  # go out of the current directory to avoid picking up local mplib (need global)
python3.11 -m pdoc -o $current_dir/docs mplib
cd $current_dir
