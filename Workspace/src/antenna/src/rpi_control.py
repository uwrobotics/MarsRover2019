#!/usr/bin/env python

import serial
import rospy
import time
from gpiozero import DigitalOutputDevice
from gpiozero import PWMOutputDevice
from std_msgs.msg import Int32
from std_msgs.msg import Float32

from numpy import interp


ser = serial.Serial("/dev/ttyACM0", 9600)
ser.flushInput()
# MOTOR_DIRECTION = DigitalOutputDevice(21)
# MOTOR_SPEED = PWMOutputDevice(18)
# DATA_ENABLE = DigitalOutputDevice(17)

# enumurate CW, CCW directions -> CW = 0, CCW = 1
class Directions:
    CW, CCW = range(2)

# get angle from antenna encoder... change as necessary and return angle in degrees
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

def update_angle(msg):
    global target_angle

    if (msg->data >= 0)
        target_angle = msg->data;
    else
        target_angle = 360 + msg->data;

    rospy.loginfo("target_angle: %f", target_angle)

def format_msg(speed, direction, valid):
    # don't move motor
    if (valid == 0): 
        output = "0"
    else:
        output = 1 + direction + speed
        output = str(output)
    return output


if __name__ == '__main__':
    #pub = rospy.Publisher('/antenna/encoder', Int32, queue_size=10)

    # get target angle from calc angle node
    sub = rospy.Subscriber('/antenna/rover_angle_from_east', Float32, update_angle)
    rospy.loginfo("Starting rpi_controller node")
    rospy.init_node('pi_controller')
    rate = rospy.Rate(20) # 20 Hz

    turn_dir = None

    # Update speed based on current angle
    while not rospy.is_shutdown():

        # store angle of antenna from encoder
        current_antenna_angle = read_serial()

        rospy.loginfo("Current rover angle: %f   Target angle: %f", current_antenna_angle, target_angle)

        angle_difference = target_angle - current_antenna_angle

        # only move if angle difference is significant
        if (abs(angle_difference) > 1):
            speed = min(((abs(angle_difference)/360)*10), 0.8) # determine speed based on difference in angles

            # map speed from 0-1 range to 0-255 range
            speed = interp(speed, [0, 1], [0, 255])

            # figure out which direction is quicker to turn in to reach target angle
            if (current_antenna_angle < target_angle):
                if ((target_angle - current_antenna_angle) > 180):
                    turn_dir = CW
                else:
                    turn_dir = CCW
            else:
                if ((antenna_position - target_angle) > 180):
                    turn_dir = CCW
                else:
                    turn_dir = CW

            # # turn motor accordingly
            # if (turn_dir == CW):
            #     MOTOR_DIRECTION.off()
            # else:
            #     MOTOR_DIRECTION.on()

            # MOTOR_SPEED.value = speed

            # send info to arduino
            output = format_msg(speed, turn_dir, 1)
        else:
 #           rospy.loginfo("Motor stopping")
            # MOTOR_SPEED.value = 0
            output = format_msg(0, 0, 0)
        ser.write(output)
        rate.sleep()
