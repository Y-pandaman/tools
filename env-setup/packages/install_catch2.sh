#!/usr/bin/env bash

set -ex

CATCH2_VERSION=${1:-v3.2.1}
NUM_THREADS=${2:-1}

# get latest source code
cd /tmp &&
  git clone https://github.com/catchorg/Catch2 &&
  cd Catch2 &&
  git checkout ${CATCH2_VERSION}

# build and install
cd /tmp/Catch2 &&
  mkdir build &&
  cd build &&
  cmake .. &&
  make install -j${NUM_THREADS}
