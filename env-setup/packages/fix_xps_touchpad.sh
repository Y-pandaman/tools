#!/usr/bin/env bash

if [ -z $COMMON_SOURCED ]; then
  source include/common.sh
fi

echo "Enable the HWE kernel"
sudo apt-get install --install-recommends linux-generic-hwe-16.04 xserver-xorg-hwe-16.04

echo "Install the new libinput driver"
sudo apt-get install xserver-xorg-input-libinput-hwe-16.04 libinput-tools xorg-input-abi-24

echo "Verify if your system has the dual mouse drivers "
xinput list

echo "Please see the following tutorial for next steps"
echo "Precision / XPS: Ubuntu General Touchpad / Mouse Issue Fix"
echo "https://www.dell.com/support/article/us/en/04/sln308258/precision-xps-ubuntu-general-touchpad-mouse-issue-fix?lang=en"

