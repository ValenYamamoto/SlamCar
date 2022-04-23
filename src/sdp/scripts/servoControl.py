#!/usr/bin/env python3
from __future__ import print_function

from sdp.srv import ServoDataResponse, ServoData, PWMControllerData, PWMControllerDataResponse

from std_msgs.msg import Int32
import rospy

SERVO_MAX_ANGLE = 180
SERVO_MIN_ANGLE = 0   #TODO: CHeck these
SERVO_CHANNEL = 15
NODE_NAME= "Servo_controller"
DC_MAX = 6200
DC_MIN = 4200
RANGE = SERVO_MAX_ANGLE
scale = (DC_MAX-DC_MIN)/180.0
dc_center = 0
dc_offset = 0
def handle_servo_control_request(req):
    angle = req.angle
    if angle >= SERVO_MIN_ANGLE and angle <= SERVO_MAX_ANGLE:
        print("I got a valid servo control angle!, setting servo...")
        pwm_dc = angle_to_duty_cycle(angle)
        err = pwm_control_client(pwm_dc, SERVO_CHANNEL)
        if(err > 0):
            print("But the PWM Controller returned an error :(")
        return ServoDataResponse(err)
    else:
        print("Could not set servo, angle OOB")
        return ServoDataResponse(3)

def handle_servo_calibration_update(data):
    print("Just got a recalibration update")
    recalibrate()

def angle_to_duty_cycle(angle):
    #TODO: Implement... Temp solution
    # return 0x3FFF
    dc = angle * (scale) + DC_MIN + dc_offset
    return int(dc)

def servo_control_server():
    rospy.init_node(NODE_NAME)
    recalibrate()
    rospy.Subscriber("steeringCalibUpdate", Int32, handle_servo_calibration_update)
    s = rospy.Service('servo_control_srv', ServoData, handle_servo_control_request)
    print(NODE_NAME + "is ready to recieve servo control commands")
    rospy.spin()

def pwm_control_client(dc, ch):
    rospy.wait_for_service('pwm_control_srv')
    try:
        pwm_control_req = rospy.ServiceProxy('pwm_control_srv', PWMControllerData)
        pwm_control_resp = pwm_control_req(dc, ch)
        return pwm_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def read_calibration_data():
    calib_file = open("/home/jks10/catkin_ws/src/f1tenth_simulator/scripts/.steering_calibration_data")
    #calib_file = open('.steering_calibration_data')
    data = calib_file.read()
    data = int(data)
    calib_file.close()
    return data

def recalibrate():
    global dc_center
    global dc_offset
    #c = read_calibration_data()
    #dc_center = c
    #dc_offset = dc_center - (90 * (scale) + DC_MIN)
    #print("new offset is " + str(dc_offset))
    print("Steering calib is current disabled")
if __name__ == "__main__":
    servo_control_server()
