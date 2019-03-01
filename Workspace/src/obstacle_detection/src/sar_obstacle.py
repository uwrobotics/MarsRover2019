#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from obstacle_detection.msg import obstacleDataArray
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import MeanShift
#import open3d
import numpy as np
import ros_numpy
import pptk

last_msg = None
pub = None

def callback(data):
    global last_msg
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo("Got a message")
    #last_msg = data
    #xyz_array = ros_numpy.point_cloud2.get_xyz_points(data)
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    print xyz_array.shape
    print(xyz_array)
    xyz_array = ros_numpy.point_cloud2.get_xyz_points(xyz_array)
    print xyz_array.shape
    print(xyz_array)
    print(np.min(xyz_array, axis=0))
    print(np.max(xyz_array, axis=0))
    #v = pptk.viewer(xyz_array)
    #v.wait()
    np.save("/home/tom/Desktop/pc.npy", xyz_array)

    xyz_array = xyz_array[:, (0,2,1)]
    xyz_array[:, 2] *= -1
    xyz_array[:, 2] -= np.min(xyz_array[:,2])

    print 'done preprocessing'

    xyz_array = xyz_array[np.where(xyz_array[:, 2] > 0.1)]
    np.save("/home/tom/Desktop/pc_pre.npy", xyz_array)

    #xyz_array = xyz_array[np.where(xyz_array[:, 2] > 1.0)]
    #print xyz_array.shape
    #print xyz_array


#def doDetection():
#    #global last_msg



def obstacle_detection():
    global pub

    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/obstacle_detection/obstacle_list', obstacleDataArray, queue_size=1)
    sub = rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, callback)

    #rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
    #    rospy.loginfo('looping')
    #    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_detection()
    except rospy.ROSInterruptException:
        pass
