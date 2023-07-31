#!/usr/bin/env bash

set -ex

USE_MIRROR=${1:-"true"}

if [ "x${USE_MIRROR}" = "xtrue" ]; then
    echo "Use mirrors"
    apt-get update &&
        apt-get install -y --no-install-recommends ca-certificates &&
        apt-get clean &&
        rm -rf /tmp/* /var/lib/apt/lists/* /var/tmp/*
    sed -i 's/ports.ubuntu.com/mirrors.sjtug.sjtu.edu.cn/g' /etc/apt/sources.list
    sed -i 's/archive.ubuntu.com/mirrors.sjtug.sjtu.edu.cn/g' /etc/apt/sources.list
    sed -i 's/archive.canonical.com/mirrors.sjtug.sjtu.edu.cn/g' /etc/apt/sources.list
    sed -i 's/security.ubuntu.com/mirrors.sjtug.sjtu.edu.cn/g' /etc/apt/sources.list
    sed -i 's/http:\/\/mirrors.sjtug.sjtu.edu.cn/https:\/\/mirrors.sjtug.sjtu.edu.cn/g' /etc/apt/sources.list
fi
