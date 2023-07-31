#!/usr/bin/env bash

set -ex

CUDA_VERSION=${1:-"11.6.2"}
TRT_VERSION=${2:-"8.4.3.1"}
OS_VERSION=${3:-"ubuntu18.04"}
TARGETARCH=${4:-"amd64"}

# Check args
OS_PATH_NAME="${OS_VERSION}"
if [[ "${OS_VERSION}" =~ .*\..* ]]; then
    # delete the dot in os string
    OS_PATH_NAME="${OS_VERSION//./}"
fi

# Install requried libraries
apt-get update && apt-get install -y --no-install-recommends software-properties-common
add-apt-repository ppa:ubuntu-toolchain-r/test
apt-get update && apt-get install -y --no-install-recommends \
    libcurl4-openssl-dev \
    wget \
    git \
    pkg-config \
    sudo \
    ssh \
    libssl-dev \
    pbzip2 \
    pv \
    bzip2 \
    unzip \
    devscripts \
    lintian \
    fakeroot \
    dh-make \
    build-essential

# Install python3
apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    python3-wheel &&
    cd /usr/local/bin &&
    ln -sf /usr/bin/python3 python &&
    ln -sf /usr/bin/pip3 pip

# Install TensorRT
if [ "${CUDA_VERSION}" = "10.2" ]; then
    v="${TRT_VERSION%.*}-1+cuda${CUDA_VERSION}" &&
        apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/${OS_PATH_NAME}/$(uname -m)/3bf863cc.pub &&
        apt-get update &&
        apt-get install -y --no-install-recommends \
            libnvinfer8=${v} \
            libnvonnxparsers8=${v} \
            libnvparsers8=${v} \
            libnvinfer-plugin8=${v} \
            libnvinfer-dev=${v} \
            libnvonnxparsers-dev=${v} \
            libnvparsers-dev=${v} \
            libnvinfer-plugin-dev=${v} \
            python3-libnvinfer=${v}
else
    v="${TRT_VERSION%.*}-1+cuda${CUDA_VERSION%.*}" &&
        apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/${OS_PATH_NAME}/$(uname -m)/3bf863cc.pub &&
        apt-get update &&
        sudo apt-get install -y --no-install-recommends \
            libnvinfer8=${v} \
            libnvonnxparsers8=${v} \
            libnvparsers8=${v} \
            libnvinfer-plugin8=${v} \
            libnvinfer-dev=${v} \
            libnvonnxparsers-dev=${v} \
            libnvparsers-dev=${v} \
            libnvinfer-plugin-dev=${v} \
            python3-libnvinfer=${v}
fi

# Clean
apt-get clean &&
    rm -rf /tmp/* /var/lib/apt/lists/* /var/tmp/*
