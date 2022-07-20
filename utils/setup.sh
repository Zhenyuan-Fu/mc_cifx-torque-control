#!/usr/bin/env sh

sudo modprobe uio_netx
for i in `seq 0 7`
do
  sudo cpufreq-set -c $i -g performance
done
