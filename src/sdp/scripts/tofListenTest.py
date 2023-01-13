#!/usr/bin/python3
import rospy

from slam import SLAMContext
from sdp.msg import ToFData
from utils import create_observations, read_yaml_file

def callback(data):
    global ctx
    observations = create_observations(ctx, data.sensor_data)
    print(data.sensor_data)
    print(observations)
    print()

def listener():
    rospy.init_node("tof_listener")
    rospy.Subscriber('/tof_data', ToFData, callback)
    rospy.spin()

if __name__ == "__main__":
    params_dict = read_yaml_file("/home/sdp10/catkin_ws/src/sdp/scripts/params.yaml")
    ctx = SLAMContext.init_from_dict(params_dict)
    listener()
