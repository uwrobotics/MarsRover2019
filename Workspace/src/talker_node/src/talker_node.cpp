/* this is a demo

*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
int main(int argc, char* argv[]){
    //main() makes it an exe when compiled
    // must call ros init before calling any other ros f'ns
    ros::init(argc, argv, "talker_node");
    // now we can use any other fn in roscpp package
    //make a node
    ros::NodeHandle node_handle;
    //control rate
    ros::Rate loop_rate(1);
    //make a publisher obj
    ros::Publisher pub;
    pub= node_handle.advertise<std_msgs::Int32>("chatter",1);
    int i=0;
    while(ros::ok){
        i++;
        //run until ctrl+c is entered. ros::ok listens for the escape sequence
        //create message instance
        std_msgs::Int32 msg;
        //set data field of msg
        msg.data=i;
        pub.publish(msg);
        loop_rate.sleep();


    }
return 0;





}
