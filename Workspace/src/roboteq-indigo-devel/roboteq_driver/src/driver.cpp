/**
Software License Agreement (BSD)

\file      driver.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
           Mike Irvine <mirvine@clearpathrobotics.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "roboteq_driver/controller.h"
#include "roboteq_driver/channel.h"

#include "ros/ros.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  std::string port_l = "/dev/ttyUSB0";
  std::string port_r = "/dev/ttyUSB1";
  std::string port_b = "/dev/ttyUSB2";
  int32_t baud = 115200;
  nh.param<std::string>("port_left", port_l, port_l);
  nh.param<std::string>("port_right", port_r, port_r);
  nh.param<std::string>("port_back", port_b, port_b);
  nh.param<int32_t>("baud", baud, baud);

  // Interface to motor controller.
  roboteq::Controller controller_left(port_l.c_str(), baud);
  roboteq::Controller controller_right(port_r.c_str(), baud);
  roboteq::Controller controller_back(port_b.c_str(), baud);

  // Setup channels.
  if (nh.hasParam("channels")) {
    XmlRpc::XmlRpcValue channel_namespaces;
    nh.getParam("channels", channel_namespaces);
    ROS_ASSERT(channel_namespaces.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(channel_namespaces.size() == 6)
    for (int i = 0; i < channel_namespaces.size(); ++i) 
    {
      ROS_ASSERT(channel_namespaces[i].getType() == XmlRpc::XmlRpcValue::TypeString);
//      controller.addChannel(new roboteq::Channel(1 + i, channel_namespaces[i], &controller));
    }
    controller_left.addChannel(new roboteq::Channel(1 + 0, channel_namespaces[0], &controller_left));
    controller_left.addChannel(new roboteq::Channel(1 + 1, channel_namespaces[1], &controller_left));
    controller_right.addChannel(new roboteq::Channel(1 + 0, channel_namespaces[2], &controller_right));
    controller_right.addChannel(new roboteq::Channel(1 + 1, channel_namespaces[3], &controller_right));
    controller_back.addChannel(new roboteq::Channel(1 + 0, channel_namespaces[4], &controller_back));
    controller_back.addChannel(new roboteq::Channel(1 + 1, channel_namespaces[5], &controller_back));
    ROS_INFO("MULTIPLE CHANNELS");
  } else {
    // Default configuration is a single channel in the node's namespace.
    controller_left.addChannel(new roboteq::Channel(1, "~", &controller_left));
    ROS_INFO("SINGLE CHANNEL");
  } 

  // Attempt to connect and run.
  while (ros::ok()) {
    ROS_DEBUG("Attempting connection to %s at %i baud.", port_l.c_str(), baud);
    controller_left.connect();
    ROS_DEBUG("Attempting connection to %s at %i baud.", port_r.c_str(), baud);
    controller_right.connect();
    ROS_DEBUG("Attempting connection to %s at %i baud.", port_b.c_str(), baud);
    controller_back.connect();
    if (controller_left.connected() && controller_right.connected() && controller_back.connected()) {
      ros::AsyncSpinner spinner(1);
      spinner.start();
      while (ros::ok()) {
        controller_left.spinOnce();
        controller_right.spinOnce();
        controller_back.spinOnce();
      }
      spinner.stop();
    } else {
      ROS_DEBUG("Problem connecting to serial device.");
      ROS_ERROR_STREAM_ONCE("Problem connecting to ports " << port_l << " or " << port_r << " or " << port_b << ". Trying again every 1 second.");
      sleep(1);
    }  
  }

  return 0;
}
