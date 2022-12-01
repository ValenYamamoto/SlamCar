#!/usr/bin/env python

from __future__ import print_function

from sdp.srv import MotorDataResponse, MotorData, PWMControllerData, PWMControllerDataResponse
import rospy
from std_msgs.msg import Int32


MAX_SPD_DC = 2900
MIN_SPD_DC = 4850
MOTOR_CHANNEL = 0
MAX_SPD = 100
MIN_SPD = 0
NODE_NAME = "Motor_controller"

# SPD_TO_DC_SCALE = None
SPD_TO_DC_SCALE = (MIN_SPD_DC - MAX_SPD_DC)/MAX_SPD

locked = False

def handle_motor_control_request(req):
    if locked == False:
        speed_percent = req.speed_percent
        rev = req.is_reverse
        if speed_percent >= MIN_SPD and speed_percent <= MAX_SPD:
            if rev:
                return MotorDataResponse(100) #Unimplemented Error
            else:
                print("I got a valid motor speed!, setting motor...")
                pwm_dc = speed_percent_to_duty_cycle(speed_percent)
                err = pwm_control_client(pwm_dc, MOTOR_CHANNEL)
                if(err > 0):
                    print("But the Motor Controller returned an error :(")
                return MotorDataResponse(err)
        else:
            print("Could not set motors, values OOB")
            return MotorDataResponse(3)
    print("locked")
    pwm_dc = speed_percent_to_duty_cycle(0)
    err = pwm_control_client(pwm_dc, MOTOR_CHANNEL)
    return MotorDataResponse(err)

def speed_percent_to_duty_cycle(speed):
    dc = MIN_SPD_DC - (speed * SPD_TO_DC_SCALE)



    # return int(dc)
    return int(dc)

def handle_connection_update(data):
    global locked
    if(data.data == 1):
        locked = False
    else:
        locked = True 
        pwm_dc = speed_percent_to_duty_cycle(0)
        err = pwm_control_client(pwm_dc, MOTOR_CHANNEL)



def motor_control_server():
    global SPD_TO_DC_SCALE
    rospy.init_node(NODE_NAME)
    SPD_TO_DC_SCALE = (MIN_SPD_DC - MAX_SPD_DC)/MAX_SPD

    s = rospy.Service('motor_control_srv', MotorData, handle_motor_control_request)
    rospy.Subscriber('checkDisconnect', Int32, handle_connection_update)

    print(NODE_NAME + "is ready to recieve motor control commands")
    err = pwm_control_client(0, MOTOR_CHANNEL)
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
