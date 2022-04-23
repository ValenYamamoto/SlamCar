#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    rospy.loginfo(rospy.get_caller_id() + "JUST GOT LASER SCAN")
    ls_info = "angle_min:" + str(data.angle_min) + " angle_max:" + str(data.angle_max) + " angle_inc:" + str(data.angle_increment) + "\n" + " range_min:" + str(data.range_min) + " range_max:" + str(data.range_max) + "\n"
    rospy.loginfo(ls_info)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('testLIDARlistenerpynode', anonymous=True)

    rospy.Subscriber('scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
