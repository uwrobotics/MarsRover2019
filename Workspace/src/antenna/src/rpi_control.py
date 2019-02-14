#!/usr/bin/env python

import serial
import rospy
import time
from gpiozero import DigitalOutputDevice
from gpiozero import PWMOutputDevice
from std_msgs.msg import Int32
from std_msgs.msg import Float32


ser = serial.Serial("/dev/ttyACM0")
MOTOR_DIRECTION = DigitalOutputDevice(21)
MOTOR_SPEED = PWMOutputDevice(18)
DATA_ENABLE = DigitalOutputDevice(17)
rover_angle = 0

def read_serial():
    DATA_ENABLE.on()
    data = ser.readline()
    rospy.loginfo("serial data: %s", data)
    try:
        data = float(data)/1024*360 # 10 bit analog signal
    except:
        rospy.loginfo("cannot convert exception")
        data = 0
    DATA_ENABLE.off()
    return data

def update_angle(float_msg):
    global rover_angle
    rover_angle = float_msg.data
    rospy.loginfo("rover_angle: %f", rover_angle)

if __name__ == '__main__':
    #pub = rospy.Publisher('/antenna/encoder', Int32, queue_size=10)
    sub = rospy.Subscriber('/antenna/rover_angle_from_east', Float32, update_angle)
    rospy.loginfo("Starting rpi_controller node")
    rospy.init_node('pi_controller')
    rate = rospy.Rate(20) # 20 Hz

    # Update speed based on current angle
    while not rospy.is_shutdown():
        current_antenna_angle = read_serial()
        angle_difference = rover_angle - current_antenna_angle
#        rospy.loginfo("angle difference: %f", angle_difference)
        #rospy.loginfo("rover_angle_in_computation: %f", rover_angle)
        if (abs(angle_difference) > 1):
            speed = min(((abs(angle_difference)/360)*10), 0.8)
#            rospy.loginfo("speed: %f", speed)
  #          rospy.loginfo("rover_angle: %f", rover_angle)
  #          rospy.loginfo("current_antenna_angle: %f", current_antenna_angle)
  #          rospy.loginfo("angle_difference: %f", angle_difference)

            turn_option_1 = (current_antenna_angle - rover_angle)
            turn_option_2 = (current_antenna_angle - (360 - rover_angle))

   #         rospy.loginfo("turn_option_1: %f", turn_option_1)
   #         rospy.loginfo("turn_option_2: %f", turn_option_2)
            turn_angle = turn_option_1 # make a guess
            if (abs(turn_option_1) > abs(turn_option_2)):
                turn_angle = turn_option_2
                if (turn_angle > 0):
                    # turn clockwise,
                    MOTOR_DIRECTION.off()
                else:
                    MOTOR_DIRECTION.on()

            MOTOR_SPEED.value = speed
        else:
 #           rospy.loginfo("Motor stopping")
            MOTOR_SPEED.value = 0
        rate.sleep()

