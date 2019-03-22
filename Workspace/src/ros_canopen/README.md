ros_canopen
===========

Canopen implementation for ROS.

Adopted from ros-industrial/ros_canopen

Usage:  
First, launch the socketcan_bridge.launch for 2 way communication, or socketcan_to_topic / topic_to_socketcan for 1 way receive / transmit.  
Then the node:  
1, Subscribes to topic "CAN_transmitter", transmit received can_msgs to the can bus;  
2, Publihses to topics as listed in ros_canopen/socketcan_bridge/config/can_config.yaml, 
note that only ids listed in can_ids will be received and published.
