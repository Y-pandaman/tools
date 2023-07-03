#!/bin/bash

set -x

# Start your program in the background
"$@" &

# Get its PID
PID=$!

N=10  # Number of latest records to keep for averaging, modify as needed

# Monitor CPU and Memory
while sleep 1; do
    CPU=$(ps -p ${PID} -o %cpu= | awk '{print $1}')
    MEM=$(ps -p ${PID} -o rss= | awk '{print $1/1024}')  # rss is in KB, convert to MB
    echo "${CPU} ${MEM}"
done | awk -v N=$N '{ cpu[NR % N] = $1; mem[NR % N] = $2; sum_cpu = sum_mem = 0; for (i in cpu) { sum_cpu += cpu[i]; sum_mem += mem[i]; } print "Average CPU: " sum_cpu / N ", Average MEM: " sum_mem / N " MB"; }'

# Wait for the program to finish
wait ${PID}
