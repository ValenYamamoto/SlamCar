#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import getch
from sdp.srv import *
from std_msgs.msg import Int32

ANGLE_STEP = 10
ANGLE_STEP_BIG = 50

MAX_SPD = 100
MIN_SPD = 0
SPD_STEP = 5
DEFAULT_ANGLE_DC = 5520 

SERVO_CHANNEL = 15
def notifyNewCalibration():
    pub = rospy.Publisher('steeringCalibUpdate', Int32, queue_size=10)
    rospy.init_node('steering_calibrator', anonymous=True)
    pub.publish(1)

def servo_control_client(a, ch):
    rospy.wait_for_service('servo_control_srv')
    try:
        servo_control_req = rospy.ServiceProxy('servo_control_srv', ServoData)
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

def pwm_control_client(dc, ch):
    rospy.wait_for_service('pwm_control_srv')
    try:
        pwm_control_req = rospy.ServiceProxy('pwm_control_srv', PWMControllerData)
        pwm_control_resp = pwm_control_req(dc, ch)
        return pwm_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    cur_angle = 90
    cur_angle_dc = DEFAULT_ANGLE_DC
    cur_spd = 0
    print("Welcome to the calibration wizard, adjust steering until car is driving straight, then press y")
    while(True):
        err = 0
        kp = getch.getch()
        if (kp == "a"):
            cur_angle_dc += ANGLE_STEP
            pwm_control_client(cur_angle_dc, SERVO_CHANNEL)
        elif (kp == "d"):
            cur_angle_dc -= ANGLE_STEP
            pwm_control_client(cur_angle_dc, SERVO_CHANNEL)
        elif (kp == "q"):
            cur_angle_dc += ANGLE_STEP_BIG
            pwm_control_client(cur_angle_dc, SERVO_CHANNEL)
        elif (kp == "e"):
            cur_angle_dc -= ANGLE_STEP_BIG
            pwm_control_client(cur_angle_dc, SERVO_CHANNEL)
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
        elif (kp == "y"):
            calib_file = open("/home/jks10/catkin_ws/src/f1tenth_simulator/scripts/.steering_calibration_data", 'w')
            calib_file.write(str(cur_angle_dc))
            calib_file.close()
            notifyNewCalibration()
            print("Exiting Wizard...")
            exit()
        else:
            print("Invalid Keyboard Input")
        if(err > 0):
            print("Client Returned Error: " + str(err))

        print("ANGLE_DC: " + str(cur_angle_dc))
        print("SPEED: " + str(cur_spd))
