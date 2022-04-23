#!/usr/bin/env python3
from __future__ import print_function
from std_msgs.msg import Int32, String

import sys
import rospy
import getch
from sdp.srv import *

MAX_ANGLE = 180 #TODO: Check if these are updated
MIN_ANGLE = 0
ANGLE_STEP = 18

MAX_SPD = 100
MIN_SPD = 0
SPD_STEP = 5

SERVO_CHANNEL = 15

DISABLE_BOT = False

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

def check_connect_callback(data):
    global DISABLE_BOT 
    if(data.data == 1):
        DISABLE_BOT = False
        print("CON: CONNECTED")
    elif(data.data == 0):
        DISABLE_BOT = True
        motor_control_client(0,0)
        print("CON: DISCONNECTED")
    elif(data.data == 2):
        print("CON: BAD INIT STATUS")
    else:
        print("CON: UNKNOWN STATUS")


cur_angle = 90
cur_spd = 0
err = 0
def keyboard_input_callback(data):
    global cur_angle, cur_spd, err
    
    print("Got KB INPUT")
    if not DISABLE_BOT: 
        print("NOT DIABLED")
        kp = data.data
        print(kp)
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
            if(cur_spd + SPD_STEP > MAX_SPD):
                cur_spd = MAX_SPD
            else:
                cur_spd += SPD_STEP
            err = motor_control_client(cur_spd, 0)
        elif (kp == "s"):
            if(cur_spd - SPD_STEP < MIN_SPD):
                cur_spd = MIN_SPD
            else:
                cur_spd -= SPD_STEP
            err = motor_control_client(cur_spd, 0)
        elif (kp == " "):
            cur_spd = 0
            err = motor_control_client(cur_spd, 0)
        elif (kp == "j"):
            cur_angle = 90
            err = servo_control_client(cur_angle, SERVO_CHANNEL)
        else:
            print("Invalid Keyboard Input")
        if(err > 0):
            print("Client Returned Error: " + str(err))
    else:
        cur_spd = 0


    print("ANGLE: " + str(cur_angle))
    print("SPEED: " + str(cur_spd))


def adjustSpeedFromAngle(degree):
    angleThresholds = [20]
    speed = []

    #for s, i in enumerate(speed):


# call thru opencv
def setAngle(degree):
    turnDegree = min(degree, MAX_ANGLE)
    turnDegree = max(turnDegree, MIN_ANGLE)

    cur_angle = turnDegree

    err = servo_control_client(cur_angle, SERVO_CHANNEL)

    if(err > 0):
        print("Client Returned Error: " + str(err))




if __name__ == "__main__":
    rospy.init_node('keyboardControl', anonymous=True)
    rospy.Subscriber("keyboardInputs", String, keyboard_input_callback)
#    rospy.Subscriber("checkDisconnect", Int32, check_connect_callback)
    rospy.spin()
    exit()

