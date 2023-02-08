#!/usr/bin/env python3
from __future__ import print_function

import sys
import signal
import rospy
from sdp.srv import *

MAX_ANGLE = 180 #TODO: Check if these are updated
MIN_ANGLE = 0
ANGLE_STEP = 18

MAX_SPD = 100
MIN_SPD = 0
SPD_STEP = 5

SERVO_CHANNEL = 15

def servo_control_client(a, ch):
    rospy.wait_for_service('servo_control_srv')
    try:
        servo_control_req = rospy.ServiceProxy('servo_control_srv', ServoData)
        # servo_control_req = f1tenth_simulator.srv.
        servo_control_resp = servo_control_req(angle=a)
        return servo_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def motor_control_client(s, rev):
    rospy.wait_for_service('motor_control_srv')
    try:
        motor_control_req = rospy.ServiceProxy('motor_control_srv', MotorData)
        motor_control_resp = motor_control_req(speed_percent=s, is_reverse=rev)
        return motor_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def sigint_handler(signum, frame):
    err = motor_control_client(0, 0)
    exit(1)


if __name__ == "__main__":
    angle = int(sys.argv[1])
    print(angle)
    err = servo_control_client(angle, SERVO_CHANNEL)
    #rospy.sleep(1)
    input()
    signal.signal(signal.SIGINT, sigint_handler)
    print("Starting Motors")
    err = motor_control_client(20, 0)
    rospy.sleep(2)
    print("Stopping Motors")
    err = motor_control_client(0, 0)
    rospy.sleep(1)
    err = servo_control_client(100, SERVO_CHANNEL)

