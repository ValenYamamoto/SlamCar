#!/usr/bin/python3
import rospy
from sdp.srv import *

def sensor_client():
    rospy.wait_for_service('tof_readings')
    try:
        sc = rospy.ServiceProxy('tof_readings', DistanceData)
        resp1 = sc()
        return resp1.sensor_data
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("Starting Client...")
    rospy.init_node("sensor_client")
    try:
        sc = rospy.ServiceProxy('tof_start', StartSensors)
        resp1 = sc(True)
        if resp1.err:
            print("init error")
            exit(1)
    except rospy.ServiceException as e:
        print("init exception")
        exit(1)

    #r = rospy.Rate(0.25)
    while True:
        data = sensor_client()
        s = ""
        for i in range(6):
            s += f"{data[i]:.2f} "
        rospy.loginfo(s)
        #r.sleep()
        input()
