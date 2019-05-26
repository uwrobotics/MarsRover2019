#!/bin/bash

sudo slcand -o -c -s6 /dev/serial/by-id/*CANtact*-if00 can0
sudo ifconfig can0 up
