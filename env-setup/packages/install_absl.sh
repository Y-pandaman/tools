#!/usr/bin/env bash

set -ex

ABSL_VERSION=${1:-"lts_2021_11_02"}
NUM_THREADS=${2:-1}

# get latest cmake source
cd /tmp && \
  git clone https://github.com/abseil/abseil-cpp.git && \
  cd abseil-cpp && \
  git checkout ${ABSL_VERSION}

# build cmake
mkdir build && cd build && \
  cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. && \
  make -j${MAKE_JOBS} && \
  make install && \
  cd /tmp && \
  rm -rf abseil-cpp
