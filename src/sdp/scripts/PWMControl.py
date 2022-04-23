#!/usr/bin/env python3.7

#Node for interfacing with the PWM controller

from __future__ import print_function

from sdp.srv import PWMControllerDataResponse, PWMControllerData
import rospy

import time
#from adafruit_servokit import ServoKit
from board import SCL_1, SDA_1
import busio
from adafruit_pca9685 import PCA9685

NODE_NAME = "PWM_Controller"
MAX_DC = 65535 #TODO: Check if this is correct
MIN_DC = 0
MAX_CH = 15
MIN_CH = 0

pca=None

def handle_pwm_control_request(req):
    dc = req.duty_cycle
    ch = req.channel
    if dc >= MIN_DC and dc <= MAX_DC:
        if ch >= MIN_CH and ch <= MAX_CH:
            print("SENDING PWM SET OVER i2c ch: " + str(ch) + " dc: " + str(dc))
            pwm_set(dc, ch)
            return PWMControllerDataResponse(0)
        else:
            print("Could not set PWM , channel value OOB")
            return PWMControllerDataResponse(3)
    else:
        print("Could not set PWM , duty cycle value OOB")
        return PWMControllerDataResponse(3)
    

def pwm_control_server():
    rospy.init_node(NODE_NAME)
    s = rospy.Service('pwm_control_srv', PWMControllerData, handle_pwm_control_request)
    print(NODE_NAME + "is ready to recieve PWM control commands")
    rospy.spin()

def pwm_set(dc, ch):
    pca.channels[ch].duty_cycle = dc

def init_pwm():
    global pca
    print("Init I2C and IC")
    i2c = busio.I2C(SCL_1, SDA_1)
    pca = PCA9685(i2c, address = 0x40)
    pca.frequency = 50

if __name__ == "__main__":
    init_pwm()
    pwm_control_server()
