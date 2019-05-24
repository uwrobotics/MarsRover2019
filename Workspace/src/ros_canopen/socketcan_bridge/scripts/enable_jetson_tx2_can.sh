#!/bin/sh

sudo modprobe can
sudo modprobe can-raw
sudo modprobe mttcan
sudo ip link set can0 up type can bitrate 500000
