// this node subscribes to tennis ball detection and decides which way to go to approach the ball
// if the ball is arrived (needs arrival_count_thres number of consecutive confirmation), publish to "arrived"
// if the ball is lost (needs lost_count_thres number of consecutive confirmation), publish to "ball_lost"
// else, based on the location of detected ball, turn the robot either CW (right) or CCW (left) with "angular_turn_rate" and "linear_rate"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include <tennis_ball_tracker/TennisBallTracker.h>
#include <std_srvs/SetBool.h>
#include <cmath>

class BallFollower{
public:
	BallFollower();

private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_param;
	ros::Subscriber posSub;
	ros::Subscriber imgSub;
	ros::Publisher velPub;
	ros::Publisher ballArrivedPub;
	ros::Publisher ballLostPub;
	ros::ServiceServer enabledServer;
	int imgWidth, imgHeight;
	float imagCenterX, imagCenterY;
	float radThres, angularTurnRate, linearRate;
	int arrivalCountThres, arrivalCount, lostCountThres, lostCount;
	bool imgSizeInitialized;
	bool hasBeenDetected;
	void ballPosCallback(tennis_ball_tracker::TennisBallTrackerConstPtr ball_detect);
	void imgCallback(sensor_msgs::ImageConstPtr image);
	bool enableAndCheckSrvCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp);
	bool bArrived;
	bool bEnabled;
};

BallFollower::BallFollower(){
	nh_param = ros::NodeHandle("~");
	nh_param.param("rad_thres", radThres, 50.0f);
	nh_param.param("arrival_count_thres", arrivalCountThres, 5);
	nh_param.param("lost_count_thres", lostCountThres, 5);
	nh_param.param("angular_turn_rate", angularTurnRate, 0.2f);
	nh_param.param("linear_rate", linearRate, 0.4f);
	nh_param.param("img_width", imgWidth, 10);
	nh_param.param("img_height", imgHeight, 10);

	bArrived = false;
	bEnabled = false;

	arrivalCount = lostCount = 0;
	hasBeenDetected = false;

	if(imgWidth == 0 || imgHeight == 0){
		imgSizeInitialized = false;
		imgSub = nh.subscribe("/tennis_ball_tracker/img", 1, &BallFollower::imgCallback, this);
	} else {
		imgSizeInitialized = true;
	}

	posSub = nh.subscribe("/tennis_ball_tracker/detection", 1, &BallFollower::ballPosCallback, this);
	velPub = nh.advertise<geometry_msgs::Twist>("/tennis_ball_follower/cmd_vel", 1, true);
	ballArrivedPub = nh.advertise<std_msgs::Bool>("/arrived", 1, true);
	ballLostPub = nh.advertise<std_msgs::Bool>("/ball_lost", 1, true);
	enabledServer = nh.advertiseService("/tennis_ball_follower/set_enabled", &BallFollower::enableAndCheckSrvCallback, this);
}

bool BallFollower::enableAndCheckSrvCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp) {
    bEnabled = req.data;
	if (bEnabled) {
		resp.success = bArrived;
	} else {
		resp.success = false;
		bArrived = false;
		hasBeenDetected = false;
		arrivalCount = lostCount = 0;
	}
	return true;
}

void BallFollower::ballPosCallback(tennis_ball_tracker::TennisBallTrackerConstPtr ball_detect){
	if (!imgSizeInitialized){
		return;
	}

	geometry_msgs::Twist cmd_vel_msg;
	cmd_vel_msg.angular.x = cmd_vel_msg.angular.y = 
	cmd_vel_msg.angular.z = cmd_vel_msg.linear.x = 
	cmd_vel_msg.linear.y = cmd_vel_msg.linear.z = 0;

	if(ball_detect->isDetected){
		// if the ball is detected even once, we say it has been dectected and the it's no longer "lost"
		hasBeenDetected = true;
		lostCount = 0;

		// if the radius is larger than some "gloden" threshold, we assume we have arrived, else keep going
		if (ball_detect->radius < radThres){
			cmd_vel_msg.linear.x = linearRate;

			// turn ccw if ball is at left side of the camera and turn cw if ball is at right side of the camera
			if (ball_detect->x < imagCenterX){
				cmd_vel_msg.angular.z = angularTurnRate; 
			} else if(ball_detect->x > imagCenterX){
				cmd_vel_msg.angular.z = - angularTurnRate; 
			} else {
				cmd_vel_msg.angular.z = 0;
			}
		} else {
			arrivalCount ++;
			// keep going
			cmd_vel_msg.linear.x = linearRate;
			cmd_vel_msg.angular.z = 0;
		}
	} 

	else if (!ball_detect->isDetected && hasBeenDetected){
		// if the ball is lost once, we don't say it's lost yet, need to reach the lostCountThres to be confident
		// that we have indeed lost the ball. While we shouldn't think that we have arrived
		lostCount ++; 
		arrivalCount = 0;
		// keep going
		cmd_vel_msg.linear.x = linearRate;
		cmd_vel_msg.angular.z = 0;
	}

	if (lostCount >= lostCountThres){
		// reset flags and counters
		hasBeenDetected = false;
		//lostCount = 0;

		// publish lost message
		std_msgs::Bool lost;
		lost.data = true;
		ballLostPub.publish(lost);

		// stop the robot
		cmd_vel_msg.angular.x = cmd_vel_msg.angular.y = 
		cmd_vel_msg.angular.z = cmd_vel_msg.linear.x = 
		cmd_vel_msg.linear.y = cmd_vel_msg.linear.z = 0;
	}

	else if (arrivalCount >= arrivalCountThres){
		// reset counters
		//arrivalCount = 0;

		// publish arrived message
		std_msgs::Bool arrived;
		arrived.data = true;
		ballArrivedPub.publish(arrived);

		// stop the robot
		cmd_vel_msg.angular.x = cmd_vel_msg.angular.y = 
		cmd_vel_msg.angular.z = cmd_vel_msg.linear.x = 
		cmd_vel_msg.linear.y = cmd_vel_msg.linear.z = 0;
	}

	velPub.publish(cmd_vel_msg);
}

void BallFollower::imgCallback(sensor_msgs::ImageConstPtr image){
	if (!imgSizeInitialized){
		imgWidth = image -> width;
		imgHeight = image -> height;
		imagCenterX = imgWidth / 2.0;
		imagCenterY = imgHeight / 2.0;
		imgSizeInitialized = true;
		imgSub.shutdown();
	}
}


int main(int argc, char **argv){
	ros::init(argc, argv, "tennis_ball_following_node");
	BallFollower bf;
	ros::Rate loop_rate(10);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
}