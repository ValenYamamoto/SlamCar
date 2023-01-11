#!/usr/bin/python3
import rospy
from sdp.msg import ToFData

def callback(data):
    print(data.sensor_data)

def listener():
    rospy.init_node("tof_listener")
    rospy.Subscriber('/tof_data', ToFData, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
