#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from sdp.srv import *

def motor_control_client(x):
    rospy.wait_for_service('motor_control_srv')
    try:
        mc = rospy.ServiceProxy('motor_control_srv', MotorData)
        resp1 = mc(x, 0)
        return resp1.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    print("Starting Client...")
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
        print("x is ", x)
    else:
        print(usage())
        sys.exit(1)
    print("%s = %s"%(x, motor_control_client(x)))
