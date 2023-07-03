#!/bin/bash

name='gst-launch'
ID=`ps -ef | grep $name | grep -v "$0" | grep -v "grep" | awk '{print $2}'`
echo $ID

for id in $ID
    do
		kill -9 $id
		echo 'killed '$id
	done