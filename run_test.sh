#!/bin/bash

release_dir="./LinuxRelease"

if test -z "$(g++ -o main main.cpp)"; then
          mv main ${release_dir}/Demo
          cd ${release_dir}
          bash ./run_simple_demo.sh
fi