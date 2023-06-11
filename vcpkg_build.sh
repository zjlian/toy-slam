#!/bin/bash
set -e

cmake -S . -B build \
    -DCMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"

cmake --build build/ -- -j8

./build/toy_slam