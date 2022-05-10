#!/usr/bin/env python3

import rospy
import platform
import subprocess
from std_msgs.msg import Int32

CONTROLLING_HOST = "192.168.0.240" 
NUM_CON_HOSTS = None
PING_DELAY = 1  #check if this can be a decimal
PING_TIMEOUT = 1 

INIT_SUCCESSFUL_FLAG = False
CONNECTED_STATUS = 1
DISCONNECTED_STATUS = 0
BAD_INIT_STATUS = 2
UNKNOWN_STATUS = 3

def ping (host, timeout):
    command  = ['ping', '-c', '1', '-W', str(PING_TIMEOUT) , host]
    return subprocess.call(command) == 0

def get_host_ip():
    IP_SUB = "192.168.0."
    ps = subprocess.Popen(('w'), stdout=subprocess.PIPE)
    output = subprocess.check_output(('awk', '{print $3}'), stdin=ps.stdout)
    ps.wait()
    output = output.decode()
    output = output.split('\n')
    for s in output:
        if IP_SUB in s:
            return s
    return None

def init():
    global INIT_SUCCESSFUL_FLAG, CONTROLLING_HOST
    CONTROLLING_HOST = get_host_ip()
    if CONTROLLING_HOST == None:
        print("COULD NOT FIND CONTROLLING HOST")
        INIT_SUCCESSFUL_FLAG = False
    else:
        print(f"FOUND CONTROLLING HOST @ {CONTROLLING_HOST}")
        INIT_SUCCESSFUL_FLAG = True
    

def talker():
    pub = rospy.Publisher('checkDisconnect', Int32, queue_size=10)
    rospy.init_node('disconnectChecker', anonymous=True)
    rate = rospy.Rate(PING_DELAY)
    init()
    while not rospy.is_shutdown():
        if INIT_SUCCESSFUL_FLAG:
            ping_res = ping(CONTROLLING_HOST, PING_TIMEOUT)
            if ping_res == True:
                print("PING SUCCESSFUL")
                pub.publish(CONNECTED_STATUS)
            else:
                print("PING NO REPLY :(")
                pub.publish(DISCONNECTED_STATUS)
        else:
            pub.publish(BAD_INIT_STATUS)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass




