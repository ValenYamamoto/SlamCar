#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import getch
from sdp.srv import *

MAX_ANGLE = 180 #TODO: Check if these are updated
MIN_ANGLE = 0
ANGLE_STEP = 18

MAX_SPD = 100
MIN_SPD = 0
SPD_STEP = 1

SERVO_CHANNEL = 15
CURRENT_DIRECTION = 0

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


if __name__ == "__main__":
    cur_angle = 90
    cur_spd = 0
    #global CURRENT_DIRECTION
    CURRENT_DIRECTION = 0
    err = motor_control_client(cur_spd, CURRENT_DIRECTION)
    while(True):
        err = 0
        kp = getch.getch()
        if (kp == "a"):
            if(cur_angle + ANGLE_STEP > MAX_ANGLE):
                cur_angle = MAX_ANGLE
            else:
                cur_angle += ANGLE_STEP
            err = servo_control_client(cur_angle, SERVO_CHANNEL)
        elif (kp == "d"):
            if(cur_angle - ANGLE_STEP < MIN_ANGLE):
                cur_angle = MIN_ANGLE
            else:
                cur_angle -= ANGLE_STEP
            err = servo_control_client(cur_angle, SERVO_CHANNEL)
        elif (kp == "w"):
            if (CURRENT_DIRECTION == 0):
                cur_spd = cur_spd + SPD_STEP if cur_spd + SPD_STEP <= MAX_SPD else MAX_SPD
            else: # CURRENT_DIRECTION == 1 (reverse)
                cur_spd = cur_spd - SPD_STEP
                if (cur_spd < 0):
                    CURRENT_DIRECTION = 0
                    cur_spd = abs(cur_spd)
            err = motor_control_client(cur_spd, CURRENT_DIRECTION)
        elif (kp == "s"):
            if (CURRENT_DIRECTION == 1):
                cur_spd = cur_spd + SPD_STEP if cur_spd + SPD_STEP <= MAX_SPD else MAX_SPD
            else: # CURRENT_DIRECTION == 0 (forward)
                cur_spd = cur_spd - SPD_STEP
                if (cur_spd < 0):
                    CURRENT_DIRECTION = 1
                    cur_spd = abs(cur_spd)
            err = motor_control_client(cur_spd, CURRENT_DIRECTION)
        elif (kp == " "):
            cur_spd = 0
            err = motor_control_client(cur_spd, 0)
        elif (kp == "j"):
            cur_angle = 90
            err = servo_control_client(cur_angle, SERVO_CHANNEL)
        else:
            print("Invalid Keyboard Input")
        #if(err > 0):
        #    print("Client Returned Error: " + str(err))

        print("ANGLE: " + str(cur_angle))
        print("SPEED: " + str(cur_spd))
        print("DIRECTION: " + str(CURRENT_DIRECTION))
