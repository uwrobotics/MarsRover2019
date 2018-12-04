ros_canopen
===========

Canopen implementation for ROS.

[![Build Status](https://travis-ci.org/ros-industrial/ros_canopen.svg?branch=kinetic-devel)](https://travis-ci.org/ros-industrial/ros_canopen)

The current develop branch is `kinetic-devel`, it targets ROS `kinetic` and `lunar`.
It should work with ROS `jade`, but this is not supported anymore (EOL).
The released version gets synced over to the distro branch for each release.

Usage:  
1, Subscribes to topic "CAN_transmitter", transmit received can_msgs to the can bus;  
2, Publihses to topic "CAN_receiver", publishes can messages received from can bus to this topic  
