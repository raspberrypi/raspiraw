#!/bin/bash

if [ "$1" = "" ]; then echo "format: `basename $0` [recording time in milliseconds]"; exit; fi

echo "removing /dev/shm/out.*.raw"
rm -rf /dev/shm/*

echo "creating files for timestamp and headers"
touch /dev/shm/tstamps.csv
touch /dev/shm/hd0.32k

echo "capturing frames for ${1}ms with 600fps requested"
raspiraw -md 7 -t $1 --fps 600 -y 10 --height 128 --top 0 --vinc 1F -ts /dev/shm/tstamps.csv -hd0 /dev/shm/hd0.32k -eus 1400 -sr 1 --regs "0157,d0;0158,04" -o /dev/shm/out.%04d.raw 2>/dev/null >/dev/null

echo "frame delta time[us] distribution"
cut -f1 -d, /dev/shm/tstamps.csv | sort -n | uniq -c


