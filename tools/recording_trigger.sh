#!/bin/bash

# This script reads the state of a hardcoded pin every set interval, and if the pin is read 'HIGH',
# a user-defined recording is launched. After the command is launched, the script exits.


# First the user lets us know whether to launch a preset or custom command
echo  'Please enter the recording command you wish to launch:'
echo  'If you instead want to read a preset command from a file, press key f and enter'
rec_cmd=
while [[ $rec_cmd = "" ]]; do
  read rec_cmd
done


# Read command from a file if rec_cmd == f, otherwise store input as command
if [[ $rec_cmd = "f" ]]; then
  rec_cmd=$(<preset_rec_cmd.txt)
  echo "Running preset command upon triggering: "
  echo $rec_cmd
else
  echo "running this command upon triggering: "
  echo $rec_cmd
fi


# Set desired input pin
echo "which pin would you like to read? (BCM numbering scheme) "
read input_pin
if [ ! -e /sys/class/gpio/gpio$input_pin ]; then
    echo "$input_pin" > /sys/class/gpio/export
fi
echo "pin $input_pin is set as input pin"


#reading the input pin continuously, every interval
declare -i i
while [ 1 ]; do
  status=$(< /sys/class/gpio/gpio$input_pin/value)
  #echo $status
  if [[ $status = "1" ]]; then
    i+=1
  fi
  sleep 0.1 #seconds
  if [[ $status != "1" && i>="1" ]]; then
    i=0
  fi
  if [[ $i = "5" ]]; then
    break
  fi
done


#run commmand
$rec_cmd
