#!/usr/bin/env python3
from __future__ import print_function
from std_msgs.msg import Int32, String

import sys
import rospy
import getch
from sdp.srv import *

def talker():
    rospy.init_node('keyboardListen', anonymous=True)
    pub = rospy.Publisher('keyboardInputs', String, queue_size=10)
    while not rospy.is_shutdown():
        kp = getch.getch()
        pub.publish(kp)
        print("Sent")

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
            pass
