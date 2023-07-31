#!/usr/bin/env bash

set -ex

GCC_VERSION=${1:-"9"}

# update apt repositories
apt-get update
apt-get install software-properties-common
add-apt-repository -y ppa:ubuntu-toolchain-r/test
apt-get update

# install newer gcc toolchain
apt-get install -q -y --no-install-recommends gcc-${GCC_VERSION} g++-${GCC_VERSION}

# set newly installed gcc as default
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 700 --slave /usr/bin/g++ g++ /usr/bin/g++-7
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-${GCC_VERSION} 800 --slave /usr/bin/g++ g++ /usr/bin/g++-${GCC_VERSION}
