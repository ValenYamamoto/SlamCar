#!/usr/bin/env python3
from __future__ import print_function

import sys
import signal
import rospy
from sdp.srv import *
from sdp.msg import GyroData

MAX_ANGLE = 180 #TODO: Check if these are updated
MIN_ANGLE = 0
ANGLE_STEP = 18

MAX_SPD = 100
MIN_SPD = 0
SPD_STEP = 5

SERVO_CHANNEL = 15


gyro_value = 0

def callback(data):
    global gyro_value
    gyro_value = data.z
    #print(f'{data.z}', end='\r')

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
    rospy.init_node("gyro_listener")
    rospy.Subscriber('/gyro_data', GyroData, callback)

    angle = int(sys.argv[1])
    print(angle)
    err = servo_control_client(angle, SERVO_CHANNEL)
    #rospy.sleep(1)
    input()
    signal.signal(signal.SIGINT, sigint_handler)
    r = rospy.Rate(10)
    print("Starting Motors")
    start_angle_1 = gyro_value
    err = motor_control_client(20, 0)
    rospy.sleep(1)
    start_angle = gyro_value
    rospy.sleep(1)
    end_angle = gyro_value
    while abs(gyro_value - start_angle_1) - 90 < 0:
        r.sleep()
    #rospy.sleep(2.5)
    print("Stopping Motors")
    print(end_angle - start_angle)
    print(gyro_value - start_angle_1)
    err = motor_control_client(0, 0)
    rospy.sleep(1)
    err = servo_control_client(100, SERVO_CHANNEL)

