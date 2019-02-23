#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist.h"
#include <arm_interface/ArmCmd.h>

#include <map>
#include <string>

/*
Teleop Drive Node that takes in a joy message and implements an algorithm to
scale the joystick to a drive message, and publishes it to cmd_vel.

*/

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "drive_lis_pub");

  ros::NodeHandle nh(""), nh_param("~");
  teleop_twist_joy::TeleopTwistJoy joy_teleop(&nh, &nh_param);

  // ROS_INFO("rUNNING");

  ros::spin();
}

namespace teleop_twist_joy {

/**
 * Internal members of class. This is the pimpl idiom, and allows more
 * flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link
 * TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl {
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher arm_control_pub;

  int dr_enable_button;
  int dr_enable_turbo_button;
  int ik_enable_button;
  int fk_enable_button;

  std::map<std::string, int> dr_axis_linear_map;
  std::map<std::string, double> dr_scale_linear_map;
  std::map<std::string, double> dr_scale_linear_turbo_map;

  std::map<std::string, int> dr_axis_angular_map;
  std::map<std::string, double> dr_scale_angular_map;
  std::map<std::string, double> dr_scale_angular_turbo_map;

  // Arm maps
  std::map<std::string, int> arm_gen_axes_map;
  std::map<std::string, double> arm_gen_scale_map;
  std::map<std::string, int> ik_axes_map;
  std::map<std::string, double> ik_scale_map;
  std::map<std::string, int> fk_axes_map;
  std::map<std::string, double> fk_scale_map;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle *nh, ros::NodeHandle *nh_param) {
  pimpl_ = new Impl;

  // The line below needs to be changed to publish to the MarsRover driver
  // system, currently it publishes to a turtle bot sim.
  pimpl_->cmd_vel_pub =
      nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>(
      "joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);
  pimpl_->arm_control_pub =
      nh->advertise<arm_interface::ArmCmd>("/arm_interface/arm_cmd", 1);

  nh_param->param<int>("dr_enable_button", pimpl_->dr_enable_button, 0);
  nh_param->param<int>("dr_enable_turbo_button", pimpl_->dr_enable_turbo_button,
                       5);

  if (nh_param->getParam("dr_axis_linear", pimpl_->dr_axis_linear_map)) {
    nh_param->getParam("dr_axis_linear", pimpl_->dr_axis_linear_map);
    nh_param->getParam("dr_scale_linear", pimpl_->dr_scale_linear_map);
    nh_param->getParam("dr_scale_linear_turbo",
                       pimpl_->dr_scale_linear_turbo_map);
  } else {
    nh_param->param<int>("dr_axis_linear", pimpl_->dr_axis_linear_map["x"], 1);
    nh_param->param<double>("dr_scale_linear", pimpl_->dr_scale_linear_map["x"],
                            0.5);
    nh_param->param<double>("dr_scale_linear_turbo",
                            pimpl_->dr_scale_linear_turbo_map["x"], 1.0);
  }

  if (nh_param->getParam("dr_axis_angular", pimpl_->dr_axis_angular_map)) {
    nh_param->getParam("dr_axis_angular", pimpl_->dr_axis_angular_map);
    nh_param->getParam("dr_scale_angular", pimpl_->dr_scale_angular_map);
    nh_param->getParam("dr_scale_angular_turbo",
                       pimpl_->dr_scale_angular_turbo_map);
  } else {
    nh_param->param<int>("dr_axis_angular", pimpl_->dr_axis_angular_map["yaw"],
                         0);
    nh_param->param<double>("dr_scale_angular",
                            pimpl_->dr_scale_angular_map["yaw"], 0.5);
    nh_param->param<double>("dr_scale_angular_turbo",
                            pimpl_->dr_scale_angular_turbo_map["yaw"],
                            pimpl_->dr_scale_angular_map["yaw"]);
  }

  if (!nh_param->getParam("ik_enable_button", pimpl_->ik_enable_button)) {
    ROS_ERROR("FAILED");
  }
  if (!nh_param->getParam("fk_enable_button", pimpl_->fk_enable_button)) {
    ROS_ERROR("FAILED");
  }
  if (!nh_param->getParam("arm_gen_axes", pimpl_->arm_gen_axes_map)) {
    ROS_ERROR("FAILED");
  }
  if (!nh_param->getParam("arm_gen_scales", pimpl_->arm_gen_scale_map)) {
    ROS_ERROR("FAILED");
  }
  if (!nh_param->getParam("ik_axes", pimpl_->ik_axes_map)) {
    ROS_ERROR("FAILED");
  }
  if (!nh_param->getParam("ik_scales", pimpl_->ik_scale_map)) {
    ROS_ERROR("FAILED");
  }
  if (!nh_param->getParam("fk_axes", pimpl_->fk_axes_map)) {
    ROS_ERROR("FAILED");
  }
  if (!nh_param->getParam("fk_scales", pimpl_->fk_scale_map)) {
    ROS_ERROR("FAILED");
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop drive enable button %i.",
                 pimpl_->dr_enable_button);
  ROS_INFO_COND_NAMED(pimpl_->dr_enable_turbo_button >= 0, "TeleopTwistJoy",
                      "Turbo on button %i.", pimpl_->dr_enable_turbo_button);
  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop ik enable button %i.",
                 pimpl_->ik_enable_button);
  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop fk enable button %i.",
                 pimpl_->fk_enable_button);

  for (auto &pair : pimpl_->dr_axis_linear_map) {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
                   pair.first.c_str(), pair.second,
                   pimpl_->dr_scale_linear_map[pair.first]);
    ROS_INFO_COND_NAMED(pimpl_->dr_enable_turbo_button >= 0, "TeleopTwistJoy",
                        "Turbo for linear axis %s is scale %f.",
                        pair.first.c_str(),
                        pimpl_->dr_scale_linear_turbo_map[pair.first]);
  }

  for (auto &pair : pimpl_->dr_axis_angular_map) {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
                   pair.first.c_str(), pair.second,
                   pimpl_->dr_scale_angular_map[pair.first]);
    ROS_INFO_COND_NAMED(pimpl_->dr_enable_turbo_button >= 0, "TeleopTwistJoy",
                        "Turbo for angular axis %s is scale %f.",
                        pair.first.c_str(),
                        pimpl_->dr_scale_angular_turbo_map[pair.first]);
  }

  for (auto &pair : pimpl_->arm_gen_axes_map) {
    ROS_INFO_NAMED("TeleopTwistJoy", "General arm axis %s on %i at scale %f.",
                   pair.first.c_str(), pair.second,
                   pimpl_->arm_gen_scale_map[pair.first]);
  }
  for (auto &pair : pimpl_->ik_axes_map) {
    ROS_INFO_NAMED("TeleopTwistJoy", "IK axis %s on %i at scale %f.",
                   pair.first.c_str(), pair.second,
                   pimpl_->ik_scale_map[pair.first]);
  }
  for (auto &pair : pimpl_->fk_axes_map) {
    ROS_INFO_NAMED("TeleopTwistJoy", "FK axis %s on %i at scale %f.",
                   pair.first.c_str(), pair.second,
                   pimpl_->fk_scale_map[pair.first]);
  }

  pimpl_->sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(
    const sensor_msgs::Joy::ConstPtr &joy_msg) {
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;
  arm_interface::ArmCmd arm_cmd_msg;
  arm_cmd_msg.data_points.resize(6);

  if (dr_enable_turbo_button >= 0 && joy_msg->buttons[dr_enable_turbo_button]) {
    if (dr_axis_linear_map.find("x") != dr_axis_linear_map.end()) {
      cmd_vel_msg.linear.x = joy_msg->axes[dr_axis_linear_map["x"]] *
                             dr_scale_linear_turbo_map["x"];
    }
    if (dr_axis_linear_map.find("y") != dr_axis_linear_map.end()) {
      cmd_vel_msg.linear.y = joy_msg->axes[dr_axis_linear_map["y"]] *
                             dr_scale_linear_turbo_map["y"];
    }
    if (dr_axis_linear_map.find("z") != dr_axis_linear_map.end()) {
      cmd_vel_msg.linear.z = joy_msg->axes[dr_axis_linear_map["z"]] *
                             dr_scale_linear_turbo_map["z"];
    }
    if (dr_axis_angular_map.find("yaw") != dr_axis_angular_map.end()) {
      cmd_vel_msg.angular.z = joy_msg->axes[dr_axis_angular_map["yaw"]] *
                              dr_scale_angular_turbo_map["yaw"];
    }
    if (dr_axis_angular_map.find("pitch") != dr_axis_angular_map.end()) {
      cmd_vel_msg.angular.y = joy_msg->axes[dr_axis_angular_map["pitch"]] *
                              dr_scale_angular_turbo_map["pitch"];
    }
    if (dr_axis_angular_map.find("roll") != dr_axis_angular_map.end()) {
      cmd_vel_msg.angular.x = joy_msg->axes[dr_axis_angular_map["roll"]] *
                              dr_scale_angular_turbo_map["roll"];
    }

    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
  } else if (joy_msg->buttons[dr_enable_button]) {
    if (dr_axis_linear_map.find("x") != dr_axis_linear_map.end()) {
      cmd_vel_msg.linear.x =
          joy_msg->axes[dr_axis_linear_map["x"]] * dr_scale_linear_map["x"];
    }
    if (dr_axis_linear_map.find("y") != dr_axis_linear_map.end()) {
      cmd_vel_msg.linear.y =
          joy_msg->axes[dr_axis_linear_map["y"]] * dr_scale_linear_map["y"];
    }
    if (dr_axis_linear_map.find("z") != dr_axis_linear_map.end()) {
      cmd_vel_msg.linear.z =
          joy_msg->axes[dr_axis_linear_map["z"]] * dr_scale_linear_map["z"];
    }
    if (dr_axis_angular_map.find("yaw") != dr_axis_angular_map.end()) {
      cmd_vel_msg.angular.z = joy_msg->axes[dr_axis_angular_map["yaw"]] *
                              dr_scale_angular_map["yaw"];
    }
    if (dr_axis_angular_map.find("pitch") != dr_axis_angular_map.end()) {
      cmd_vel_msg.angular.y = joy_msg->axes[dr_axis_angular_map["pitch"]] *
                              dr_scale_angular_map["pitch"];
    }
    if (dr_axis_angular_map.find("roll") != dr_axis_angular_map.end()) {
      cmd_vel_msg.angular.x = joy_msg->axes[dr_axis_angular_map["roll"]] *
                              dr_scale_angular_map["roll"];
    }

    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
  } else if (joy_msg->buttons[fk_enable_button]) {
    arm_cmd_msg.ik_status = false;
    if (arm_gen_axes_map.find("turntable") != arm_gen_axes_map.end()) {
      arm_cmd_msg.data_points[0] =
          joy_msg->axes[arm_gen_axes_map["turntable"]] *
          arm_gen_scale_map["turntable"];
    }
    if (arm_gen_axes_map.find("wristroll") != arm_gen_axes_map.end()) {
      arm_cmd_msg.data_points[4] =
          joy_msg->axes[arm_gen_axes_map["wristroll"]] *
          arm_gen_scale_map["wristroll"];
    }
    if (arm_gen_axes_map.find("claw") != arm_gen_axes_map.end()) {
      arm_cmd_msg.data_points[5] =
          joy_msg->axes[arm_gen_axes_map["claw"]] * arm_gen_scale_map["claw"];
    }
    if (fk_axes_map.find("shoulder") != fk_axes_map.end()) {
      arm_cmd_msg.data_points[1] =
          joy_msg->axes[fk_axes_map["shoulder"]] * fk_scale_map["shoulder"];
    }
    if (fk_axes_map.find("elbow") != fk_axes_map.end()) {
      arm_cmd_msg.data_points[2] =
          joy_msg->axes[fk_axes_map["elbow"]] * fk_scale_map["elbow"];
    }
    if (fk_axes_map.find("wrist") != fk_axes_map.end()) {
      arm_cmd_msg.data_points[3] =
          joy_msg->axes[fk_axes_map["wrist"]] * fk_scale_map["wrist"];
    }
  } else if (joy_msg->buttons[ik_enable_button]) {
    arm_cmd_msg.ik_status = true;
    if (arm_gen_axes_map.find("turntable") != arm_gen_axes_map.end()) {
      arm_cmd_msg.data_points[0] =
          joy_msg->axes[arm_gen_axes_map["turntable"]] *
          arm_gen_scale_map["turntable"];
    }
    if (arm_gen_axes_map.find("wristroll") != arm_gen_axes_map.end()) {
      arm_cmd_msg.data_points[4] =
          joy_msg->axes[arm_gen_axes_map["wristroll"]] *
          arm_gen_scale_map["wristroll"];
    }
    if (arm_gen_axes_map.find("claw") != arm_gen_axes_map.end()) {
      arm_cmd_msg.data_points[5] =
          joy_msg->axes[arm_gen_axes_map["claw"]] * arm_gen_scale_map["claw"];
    }
    if (ik_axes_map.find("fwd") != ik_axes_map.end()) {
      arm_cmd_msg.data_points[1] =
          joy_msg->axes[ik_axes_map["fwd"]] * ik_scale_map["fwd"];
    }
    if (ik_axes_map.find("up") != ik_axes_map.end()) {
      arm_cmd_msg.data_points[2] =
          joy_msg->axes[ik_axes_map["up"]] * ik_scale_map["up"];
    }
    if (ik_axes_map.find("theta") != ik_axes_map.end()) {
      arm_cmd_msg.data_points[3] =
          joy_msg->axes[ik_axes_map["theta"]] * ik_scale_map["theta"];
    }
  } else {
    // When enable button is released, immediately send a single no-motion
    // command
    // in order to stop the robot.
    if (!sent_disable_msg) {
      cmd_vel_pub.publish(cmd_vel_msg);
      sent_disable_msg = true;
    }
  }
}

} // namespace teleop_twist_joy
