#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from obstacle_detection.msg import obstacleDataArray
from obstacle_detection.msg import obstacleData
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import MeanShift
# import open3d
import numpy as np
import ros_numpy
import pptk
import open3d
from sklearn.cluster import DBSCAN

last_msg = None
pub = None


def callback(data):
    global last_msg
    global pub
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo("Got a message")
    # last_msg = data
    # xyz_array = ros_numpy.point_cloud2.get_xyz_points(data)
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    # print xyz_array.shape
    # print(xyz_array)
    xyz_array = ros_numpy.point_cloud2.get_xyz_points(xyz_array)
    # print xyz_array.shape
    # print(xyz_array)
    # print(np.min(xyz_array, axis=0))
    # print(np.max(xyz_array, axis=0))
    # v = pptk.viewer(xyz_array)
    # v.wait()
    np.save("/home/tom/Desktop/pc.npy", xyz_array)

    xyz_array = xyz_array[:, (0, 2, 1)]
    xyz_array[:, 2] *= -1
    xyz_array[:, 2] -= np.min(xyz_array[:, 2])

    print 'done preprocessing'

    xyz_array = xyz_array[np.where(xyz_array[:, 2] > 0.1)]
    np.save("/home/tom/Desktop/pc_pre.npy", xyz_array)

    # xyz_array = xyz_array[np.where(xyz_array[:, 2] > 1.0)]
    # print xyz_array.shape
    # print xyz_array
    pcd = open3d.PointCloud()
    pcd.points = open3d.Vector3dVector(xyz_array)
    downpcd = open3d.voxel_down_sample(pcd, voxel_size=0.15)
    downpts = np.asarray(downpcd.points)
    clustering = DBSCAN(eps=0.65, min_samples=5).fit(
        np.asarray(downpcd.points))
    # print clustering.labels_
    labels = set(clustering.labels_)
    print "clustered"
    msg = obstacleDataArray()

    for label in labels:
        print label
        if label == -1:
            continue

        pts = downpts[np.where(clustering.labels_ == label)]
        if pts.shape[0] < 15:
            continue
        # print pts
        mean = np.mean(pts, axis=0)
        obstacle = obstacleData()
        obstacle.x = mean[0]
        obstacle.z = mean[1]
        obstacle.diameter = 2 * np.linalg.norm(np.std(pts[:, :2], axis=0))
        cov = np.cov(pts[:, :2], rowvar=False)
        w, v = np.linalg.eig(cov)
        obstacle.diam_major = np.max(w)
        obstacle.diam_minor = np.min(w)
        obstacle.cov_angle = np.arctan2(v[1, np.argmax(w)], v[0, np.argmax(w)])
        if obstacle.diam_major > 6.0 or (
                obstacle.diam_major > 3 and obstacle.diam_minor > 3.0):
            continue  # bad frame

        # print obstacle.x, obstacle.z, obstacle.diameter, obstacle.diam_major,
        # obstacle.diam_minor, obstacle.cov_angle
        msg.obstacles.append(obstacle)
    print "done"
    # print msg.obstacles
    pub.publish(msg)


# def doDetection():
#    #global last_msg


def obstacle_detection():
    global pub

    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/obstacle_detection/obstacle_list',
                          obstacleDataArray, queue_size=1)
    sub = rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2,
                           callback, queue_size=1)

    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #    rospy.loginfo('looping')
    #    rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        obstacle_detection()
    except rospy.ROSInterruptException:
        pass
