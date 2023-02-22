#!/usr/bin/python3
import rospy

from slam import SLAMContext
from sdp.msg import GyroData

def callback(data):
    print(f'{data.z}', end='\r')

def listener():
    rospy.init_node("gyro_listener")
    rospy.Subscriber('/gyro_data', GyroData, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
