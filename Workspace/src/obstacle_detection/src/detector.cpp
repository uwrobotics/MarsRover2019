#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <opencv2/core.hpp>
#include <opencv2/saliency.hpp>
#include <opencv2/imgproc/imgproc.hpp>


//Inspired from: https://stackoverflow.com/a/16083336/8245487
//And : https://github.com/stereolabs/zed-ros-wrapper/blob/master/tutorials/zed_depth_sub_tutorial/src/zed_depth_sub_tutorial.cpp
class MapReader{
	public:
		MapReader(ros::NodeHandle* node):leftImage(NULL), depthMap(NULL), mapWidth(0), mapHeight(0), n(node){
			subLeftImage = n->subscribe("/zed/left/image_raw_color", 10, leftImageCallback);
			subDepthMap = n->subscribe("/zed/depth/depth_registered", 10, depthMapCallback);
		}

		void leftImageCallback(const sensor_msgs::Image::ConstPtr& msg)
		{
			leftImage = (float*)(&msg->data[0]);
			mapWidth = msg->width;
			mapHeight = msg->height;
		}
		
		void depthMapCallback(const sensor_msgs::Image::ConstPtr& msg)
		{
			depthMap = (float*)(&msg->data[0]);
		}

		float* getLeftImage(){
			return leftImage;
		}		
	
		float* getDepthMap(){
			return depthMap;
		}

		int getMapWidth(){
			return mapWidth;
		}

		int getMapHeight(){
			return mapHeight;
		}

	private:
		float* leftImage;
		float* depthMap; 
		int mapWidth;
		int mapHeight;
		ros::NodeHandle* n;
		ros::Subscriber subLeftImage;
		ros::Subscriber subDepthMap;
}

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

	ros::NodeHandler n;	
	ros::Rate r(10);
	MapReader d(&n);
	//SpinOnce Pattern for Callbacks: https://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
	while(1){
		ros::spinOnce();

		float* leftImage = d.getLeftImage();	
		//float* to cv::Mat conversion: https://stackoverflow.com/questions/39579398/opencv-how-to-create-mat-from-uint8-t-pointer
		cv::Mat leftImage(d.getMapHeight(),d.getMapWidth(), CV_32UC3, leftImage); //3 channel (RGB) data	
		//Image to Saliency: https://github.com/fpuja/opencv_contrib/blob/saliencyModuleDevelop/modules/saliency/samples/computeSaliency.cpp	


		cv::Ptr<Saliency> saliencyAlgorithm = Saliency::create("SPECTRAL_RESIDUAL");
		if( saliencyAlgorithm == NULL){
			std::cout << "ERROR in instantiation of saliency algorithm\n";
			return -1;
		}

		cv::Mat saliencyMap;
		cv::Mat binaryMap;
		if(saliencyAlgorithm->computeSaliency(image,saliencyMap)){
			saliency::StaticSaliencySpectralResidual spec;
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
		std::vector<vector<Point> > contours;
		std::vector<Vec4i> hierarchy;

		// detect edges using canny
		cv::Canny( binaryMap, canny_output, 50, 150, 3 );

		// find contours
		cv::findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

		// get the moments
		std::vector<Moments> mu(contours.size());
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
		std::vector<Point2f> mc(contours.size());
		for( int i = 0; i<contours.size(); i++){ 
			mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
		}


		r.sleep();
	}

	return 0;
}
