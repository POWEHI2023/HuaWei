#!/bin/bash

release_dir="./Release"

if test -z "$(g++-13 -O3 -o main main.cpp)"; then
          mv main ${release_dir}/demo
          cd ${release_dir}
          bash ./run_simple_demo.sh
fi