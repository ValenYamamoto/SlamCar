#!/usr/bin/python3
from __future__ import print_function

import sys
import rospy
from sdp.srv import *

import board
import adafruit_vl53l4cd



MAX_ANGLE = 180 
MIN_ANGLE = 0
ANGLE_STEP = 18

MAX_SPD = 100
MIN_SPD = 0
SPD_STEP = 5

SERVO_CHANNEL = 15


def motor_control_client(s, rev):
    rospy.wait_for_service('motor_control_srv')
    try:
        motor_control_req = rospy.ServiceProxy('motor_control_srv', MotorData)
        motor_control_resp = motor_control_req(speed_percent=s, is_reverse=rev)
        return motor_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def get_range(vl53):
    vl53.start_ranging()
    total = 0
    for i in range(3):
        while not vl53.data_ready:
            pass
        vl53.clear_interrupt()
        total += vl53.distance
    vl53.stop_ranging()
    return total / 3

if __name__ == "__main__":
    try:
        i2c = board.I2C()
        vl53 = adafruit_vl53l4cd.VL53L4CD(i2c)

        err = motor_control_client(10, 0)
        while True:
            z = get_range(vl53) 
            if z < 20 or abs(z - 20) < 1:
                break
            print("Distance:", z, "cm")
            #r.sleep()
        err = motor_control_client(0, 0)
    except:
        err = motor_control_client(0, 0)
        raise

