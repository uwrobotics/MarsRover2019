#!/usr/bin/env python

import rospy
from tennis_ball_tracker.msg import TennisBallTracker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

import cv2
import numpy as np
import imutils

# Defining lower and upper bounds of green ball in HSV
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

# greenLower = (25, 0, 0)
# greenUpper = (60, 255, 255)

# camera = cv2.VideoCapture(0)
bridge = CvBridge()

SUCCESS_MAX = 5
SUCCESS_MIN = -5

bEnabled = False
success_cnt = SUCCESS_MIN


def enableAndCheckSrvCallback(req):
    global bEnabled
    global success_cnt
    if req.data:
        bEnabled = True
        return {'success': (success_cnt > 0), 'message': 'Detection result'}
    else:
        bEnabled = False
        return {'success': False, 'message': 'Disabled tracking'}


def detectTennisBall(imageMsg):
    global bEnabled
    global success_cnt
    if not bEnabled:
        return

    frame = bridge.imgmsg_to_cv2(imageMsg, desired_encoding="bgr8")

    # Resizing the frame
    frame = imutils.resize(frame, width=600)

    # Blurring and changing colour space
    frame_blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    frame_hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)

    # Green mask
    mask = cv2.inRange(frame_hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Finding contours in the mask
    (_, contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

    ball_center = None
    tennisBalls = frame.copy()

    if contours is not None and len(contours) > 0:
        # ballDetected = True

        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        ((ball_x, ball_y), ball_radius) = cv2.minEnclosingCircle(
            largest_contour)

        # Finding the centroid
        moments = cv2.moments(largest_contour)
        ball_center = (int(moments["m10"] / moments["m00"]),
                       int(moments["m01"] / moments["m00"]))

        # Check radius constraints
        outputMsg = TennisBallTracker()    


        if ball_radius > 10:
            # Draw circle around tennis ball and point at center
            # cv2.circle(tennisBalls, (int(ball_x), int(ball_y)),
            #            int(ball_radius), (0, 0, 255), 3)
            # cv2.circle(tennisBalls, ball_center, 3, (0, 0, 255), -1)
            top_left_point_x = int(ball_x - ball_radius)
            top_left_point_y = int(ball_y - ball_radius)
            bottom_right_point_x = int(ball_x + ball_radius)
            bottom_right_point_y = int(ball_y + ball_radius)
            top_left_point = (top_left_point_x, top_left_point_y)
            bottom_right_point = (bottom_right_point_x, bottom_right_point_y)
            cv2.rectangle(tennisBalls, top_left_point, bottom_right_point,
                          (0, 255, 0), 3)
            outputMsg.x = ball_x
            outputMsg.y = ball_y
            outputMsg.radius = ball_radius
            outputMsg.isDetected = True
            success_cnt += 1
            success_cnt = min(SUCCESS_MAX, success_cnt)
        else:
            outputMsg.isDetected = False
            success_cnt -= 1
            success_cnt = max(success_cnt, SUCCESS_MIN)


        # Publish TennisBallTracker message
        detection_pub.publish(outputMsg)

        tennisBalls = cv2.cvtColor(tennisBalls, cv2.COLOR_BGR2RGB)
        image_message = bridge.cv2_to_imgmsg(tennisBalls, encoding="rgb8")
        img_pub.publish(image_message)


if __name__ == '__main__':
    rospy.init_node('tennis_ball_tracker', anonymous=True)

    rospy.Subscriber("/zed_node/rgb/image_rect_color", Image, detectTennisBall)
    img_pub = rospy.Publisher("/tennis_ball_tracker/image", Image, queue_size=1)
    detection_pub = rospy.Publisher("/tennis_ball_tracker/detection", TennisBallTracker, queue_size=1)
    enabled_srv = rospy.Service("/tennis_ball_tracker/set_enabled", SetBool, enableAndCheckSrvCallback)

    while not rospy.is_shutdown():
        rospy.spin()
