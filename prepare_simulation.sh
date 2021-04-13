#!/usr/bin/env bash

# compile NTRT and dependencies
pushd external/ntrt
./setup.sh
popd

# compile simulation apps
pushd simulation
rm -rf apps build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j2
popd
