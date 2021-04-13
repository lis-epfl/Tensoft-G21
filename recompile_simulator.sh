#!/usr/bin/env bash

pushd simulation
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j2
popd
