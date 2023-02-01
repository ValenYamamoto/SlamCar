#!/usr/bin/env python

from __future__ import print_function

from sdp.srv import MotorDataResponse, MotorData, PWMControllerData, PWMControllerDataResponse
import rospy
from std_msgs.msg import Int32
import time

MAX_SPD_DC = 2900
MIN_SPD_DC = 7250
MOTOR_CHANNEL = 0
MAX_SPD = 100
MIN_SPD = 0
NODE_NAME = "Motor_controller"

FWD_MIN = 4790
FWD_MAX = FWD_MIN - 1200 #2000
FWD_RANGE = abs(FWD_MAX - FWD_MIN)
FWD_SCALE = -(FWD_RANGE/100) # lower value==>faster when forward
FWD_OOB = 1000

REV_MIN = 5120 #5200
REV_MAX = REV_MIN + 1200 #9400
REV_RANGE = abs(REV_MAX - REV_MIN)
REV_SCALE = REV_RANGE/100
REV_OOB = 10000

STOP = (REV_MIN + FWD_MIN)/2

#initial direction assumed to be forward
CURRENT_DIRECTION = 0 # 0 is forward, 1 is reverse

# SPD_TO_DC_SCALE = None
SPD_TO_DC_SCALE = (MIN_SPD_DC - MAX_SPD_DC)/MAX_SPD

locked = False

def handle_motor_control_request(req):
    speed_percent = req.speed_percent
    direction = req.is_reverse
    global CURRENT_DIRECTION

    if locked == True:
        print("locked")
        err = pwm_control_client(STOP, MOTOR_CHANNEL)
        return MotorDataResponse(err)
    if speed_percent < MIN_SPD or speed_percent > MAX_SPD:
        print("Could not set motors, values OOB")
        return MotorDataResponse(3)

    if (direction != CURRENT_DIRECTION):
        CURRENT_DIRECTION = direction
        #stop car first
        err = pwm_control_client(STOP, MOTOR_CHANNEL)
        #maybe a time.sleep() here
        time.sleep(1)       
        if (CURRENT_DIRECTION == 1): # reverse
            err = pwm_control_client(REV_OOB, MOTOR_CHANNEL)
            time.sleep(1)
        else:
            err = pwm_control_client(FWD_OOB, MOTOR_CHANNEL)
            time.sleep(1)
        
        
    
    if (speed_percent == 0):
        err = pwm_control_client(STOP, MOTOR_CHANNEL)
        time.sleep(1)
    else:
        pwm_val = STOP
        if (CURRENT_DIRECTION == 0): #FORWARD
            pwm_val = FWD_MIN + speed_percent*FWD_SCALE
            err = pwm_control_client(pwm_val, MOTOR_CHANNEL)
        else: #REVERSE
            pwm_val = REV_MIN + speed_percent*REV_SCALE
            err = pwm_control_client(pwm_val, MOTOR_CHANNEL)
        print("REV: " + str(CURRENT_DIRECTION) + " PWM: " + str(pwm_val))

    
    if(err > 0):
        print("motor control ret error")
    return MotorDataResponse(err)

    

def handle_connection_update(data):
        global locked
        if(data.data == 1):
            locked = False
        else:
            locked = True
            err = pwm_control_client(STOP, MOTOR_CHANNEL)



def motor_control_server():
    global SPD_TO_DC_SCALE
    rospy.init_node(NODE_NAME)
    SPD_TO_DC_SCALE = (MIN_SPD_DC - MAX_SPD_DC)/MAX_SPD

    s = rospy.Service('motor_control_srv', MotorData, handle_motor_control_request)
    rospy.Subscriber('checkDisconnect', Int32, handle_connection_update)

    print(NODE_NAME + "is ready to recieve motor control commands")
    err = pwm_control_client(STOP, MOTOR_CHANNEL)
    rospy.spin()

def pwm_control_client(dc, ch):
    rospy.wait_for_service('pwm_control_srv')
    try:
        pwm_control_req = rospy.ServiceProxy('pwm_control_srv', PWMControllerData)
        pwm_control_resp = pwm_control_req(dc, ch)
        return pwm_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    motor_control_server()
