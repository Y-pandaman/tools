#!/bin/bash

mkdir build
cd build
cmake ..
make -j

cp libyalk.so ../test
cp ../include/yalk/interface/yalk.h ../test

cd ../test/
mkdir build
cd build
cmake ..
make -j
./test
