#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <opencv2/core.hpp>
#include <opencv2/saliency.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../include/detector.h"
#include "obstacle_detection/obstacleDataArray.h"
#include "obstacle_detection/obstacleData.h"

//Inspired from: https://stackoverflow.com/a/16083336/8245487
//And : https://github.com/stereolabs/zed-ros-wrapper/blob/master/tutorials/zed_depth_sub_tutorial/src/zed_depth_sub_tutorial.cpp
class MapReader{
	public:
		
		MapReader(ros::NodeHandle* node): n(node){
			subLeftImage = n->subscribe("/zed/left/image_raw_color", 10, MapReader::leftImageCallback);
			subDepthMap = n->subscribe("/zed/depth/depth_registered", 10, MapReader::depthMapCallback);
			subCameraInfo = n->subscribe("/zed/rgb/camera_info", 10, MapReader::cameraInfoCallback);
			leftImage = NULL;
			mapWidth = 0;
			mapHeight = 0;
			depthMap = NULL;
			focalLength = 0.0;
		}

		static void leftImageCallback(const sensor_msgs::Image::ConstPtr& msg)
		{
			leftImage = (float*)(&msg->data[0]);
			mapWidth = msg->width;
			mapHeight = msg->height;
		}

		static void depthMapCallback(const sensor_msgs::Image::ConstPtr& msg)
		{
			depthMap = (float*)(&msg->data[0]);
		}

		static void cameraInfoCallback(const sensor_msgs::CameraInfoPtr& msg){
			focalLength = msg->K[0];	
		}

		float* getLeftImage(){
			return leftImage;
		}		

		float* getDepthMap(){
			return depthMap;
		}
		
		float getFocalLength(){
			return focalLength;
		}

		int getMapWidth(){
			return mapWidth;
		}

		int getMapHeight(){
			return mapHeight;
		}

	private:	
		static float* leftImage;
		static float* depthMap; 
		static float focalLength ;
		static int mapWidth ;
		static int mapHeight;
		ros::NodeHandle* n;
		ros::Subscriber subLeftImage;
		ros::Subscriber subDepthMap;
		ros::Subscriber subCameraInfo;
};


float* MapReader::leftImage;
float* MapReader::depthMap; 
float MapReader::focalLength;
int MapReader::mapWidth;
int MapReader::mapHeight;

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "obstacle_detection");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */

	ros::NodeHandle n;	
	ros::Rate r(10);
	MapReader d(&n);
	ros::Publisher obstaclePub = n.advertise<obstacle_detection::obstacleDataArray>("obstacles", 100);
	//SpinOnce Pattern for Callbacks: https://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
	while(ros::ok()){
		ros::spinOnce();

		float* leftImagePointer = d.getLeftImage();	
		float* depthMapPointer = d.getDepthMap();
		float focalLength = d.getFocalLength();
		ROS_INFO("Focal length: %9.6f", focalLength);
		ROS_INFO("LeftImageHeight: %d", d.getMapHeight());
		//float* to cv::Mat conversion: https://stackoverflow.com/questions/39579398/opencv-how-to-create-mat-from-uint8-t-pointer
		cv::Mat leftImage(d.getMapHeight(),d.getMapWidth(), CV_8UC3, leftImagePointer); //3 channel (RGB) data	
		cv::Mat depthMap(d.getMapHeight(),d.getMapWidth(), CV_8UC1, depthMapPointer); //1-channel data
		//Image to Saliency: https://github.com/fpuja/opencv_contrib/blob/saliencyModuleDevelop/modules/saliency/samples/computeSaliency.cpp	


		cv::Ptr<cv::saliency::Saliency> saliencyAlgorithm = cv::saliency::StaticSaliencySpectralResidual::create(/*"SPECTRAL_RESIDUAL"*/);
		if( saliencyAlgorithm == NULL){
			std::cout << "ERROR in instantiation of saliency algorithm\n";
			return -1;
		}

		cv::Mat saliencyMap;
		cv::Mat binaryMap;
		if(saliencyAlgorithm->computeSaliency(leftImage,saliencyMap)){
			cv::saliency::StaticSaliencySpectralResidual spec;
			spec.computeBinaryMap(saliencyMap, binaryMap);	
		}	
		else{
			std::cout <<"ERROR in computing saliency map from Left RGB map...\n";
			return -1;		
		}

		//Now we have the binary Saliency map...
		//locate the centroids of islands of 1s (contours) in the binary Saliency map.
		//https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
		cv::Mat canny_output;
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		// detect edges using canny
		cv::Canny( binaryMap, canny_output, 50, 150, 3 );

		// find contours
		cv::findContours( canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,cv::Point(0, 0) );

		// get the moments
		std::vector<cv::Moments> mu(contours.size());
		for( int i = 0; i<contours.size(); i++ ){ 
			mu[i] = cv::moments( contours[i], false ); 
		}

		//get the diameters
		std::vector<unsigned int> diameters(contours.size());
		for( int i = 0; i<contours.size(); i++){
			int area = cv::contourArea(contours[i], false);
			diameters[i] = sqrt(4 * area/3.14159265358979323846); //equivalent diameter of circle of same area as contour
		}

		// get the centroid of figures.
		std::vector<cv::Point2f> mc(contours.size());
		for( int i = 0; i<contours.size(); i++){ 
			mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
		}

		//vector of msgs pattern: https://answers.ros.org/question/60614/how-to-publish-a-vector-of-unknown-length-of-structs-in-ros/	
		obstacle_detection::obstacleDataArray dataArray;
 		for(int i = 0; i < mc.size(); i++){
			double depth = depthMap.at<double>(mc[i].x,mc[i].y);//get the depth of each centroid	
			double xdisplacement = (mc[i].x - d.getMapWidth()/2)*depth/focalLength; 
			obstacle_detection::obstacleData obstacle;
			obstacle.x = xdisplacement;
			obstacle.z = depth;
			obstacle.diameter = diameters[i];
 			dataArray.obstacles.push_back(obstacle);	
			ROS_INFO("num: %d", i);			
			ROS_INFO("X: %9.6f", xdisplacement);
			ROS_INFO("Z: %9.6f", depth);
			ROS_INFO("D: %9.6f", diameters[i]);
		}		

		obstaclePub.publish(dataArray);


		//r.sleep();
	}

	return 0;
}
