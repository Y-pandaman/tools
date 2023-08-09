#!/bin/bash

mkdir build
cd build
cmake ..
make -j

cp libspdlog.so ../test

cd ../test/
mkdir build
cd build
cmake ..
make -j
./test
