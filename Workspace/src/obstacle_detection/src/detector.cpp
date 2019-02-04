#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <opencv2/core.hpp>
#include <opencv2/saliency.hpp>



//Inspired from: https://stackoverflow.com/a/16083336/8245487
//And : https://github.com/stereolabs/zed-ros-wrapper/blob/master/tutorials/zed_depth_sub_tutorial/src/zed_depth_sub_tutorial.cpp
class MapReader{
	public:
	MapReader(ros::NodeHandle* node):map(NULL), mapWidth(0), mapHeight(0), n(node){
		subMap = n->subscribe("/zed/left/image_raw_color", 10, mapCallback);
	}

	void mapCallback(const sensor_msgs::Image::ConstPtr& msg)
	{
		map = (float*)(&msg->data[0]);
		mapWidth = msg->width;
		mapHeight = msg->height;
	}

	float* getMap(){
		return map;
	}		

	int getMapWidth(){
		return mapWidth;
	}

	int getMapHeight(){
		return mapHeight;
	}

	/*
	void run(){
		
	}
*/
		
	private:
	float* map;
	int mapWidth;
	int mapHeight;
	ros::NodeHandle* n;
	ros::Subscriber subMap;
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
		float* map = d.getMap();	
		//float* to cv::Mat conversion: https://stackoverflow.com/questions/39579398/opencv-how-to-create-mat-from-uint8-t-pointer
		cv::Mat image(d.getMapHeight(),d.getMapWidth(), CV_32UC3, map); //3 channel (RGB) data	
		//Image to Saliency: https://github.com/fpuja/opencv_contrib/blob/saliencyModuleDevelop/modules/saliency/samples/computeSaliency.cpp	
		
			
		cv::Ptr<Saliency> saliencyAlgorithm = Saliency::create("SPECTRAL_RESIDUAL");
		if( saliencyAlgorithm == NULL){
			std::cout << "ERROR in instantiation of saliency algorithm\n";
			return -1;
		}
		
		Mat saliencyMap;
		Mat binaryMap;
		if(saliencyAlgorithm->computeSaliency(image,saliencyMap)){
			saliency::StaticSaliencySpectralResidual spec;
			spec.computeBinaryMap(saliencyMap, binaryMap);	
		}	
		else{
			std::cout <<"ERROR in computing saliency map from Left RGB map...\n";
			return -1;		
		}
		
		//Now we have the binary Saliency map...
		//locate the islands of 1s in the binary Saliency map.
	/*
		for(int i = 0; i < binaryMap.rows; i++){
			for(int j = 0; j < binaryMap.columns; j++){
				if(binaryMap[i][j] == true){
					//traverse the "circle" along the top edge left and right to find the diameter and midpoint
					int minx = 0;
					int height = 0;	
					int a,b = 0;
					do{
							
						
					}while(i - a >= 0 && j - b >= 0 && binaryMap[i - a][j - b] == true);	
										
				}	
			}
			
		}	
	*/	
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
