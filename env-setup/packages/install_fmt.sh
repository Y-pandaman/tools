#!/usr/bin/env bash

set -ex

FMT_VERSION=${1:-"9.0.0"}
NUM_THREADS=${2:-1}

# get latest source code
cd /tmp && \
  git clone https://github.com/fmtlib/fmt && \
  cd fmt && \
  git checkout ${FMT_VERSION}

# build cmake
cmake -Bbuild -H. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
  cmake --build build -j${NUM_THREADS} && \
  cmake --build build --target install && \
  cd /tmp && \
  rm -rf fmt
