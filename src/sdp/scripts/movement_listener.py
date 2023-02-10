#!/usr/bin/env python3
from __future__ import print_function

import sys
import signal
import rospy
from sdp.srv import *
from utils import move_jetson

MAX_ANGLE = 180 #TODO: Check if these are updated
MIN_ANGLE = 0
ANGLE_STEP = 18

MAX_SPD = 100
MIN_SPD = 0
SPD_STEP = 5

SERVO_CHANNEL = 15

def sigint_handler(signum, frame):
    err = motor_control_client(0, 0)
    exit(1)


if __name__ == "__main__":
    move_jetson(0, 0)
    rospy.sleep(1)
    signal.signal(signal.SIGINT, sigint_handler)
    while True:
        speed = int(input("enter motor speed (0-100)"))
        turn = int(input("enter turn angle"))
        move_jetson(speed, turn)
