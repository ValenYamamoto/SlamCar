#!/usr/bin/python3
from sdp.msg import GyroData
import rospy
import adafruit_mpu6050
import adafruit_tca9548a
import board 
import busio
from micropython import const
from gyro import get_position
from math import pi


def init_sensors():
    i2c = board.I2C()
    mux = adafruit_tca9548a.TCA9548A(i2c)
    mux_pos = 6
    try:
        mpu = adafruit_mpu6050.MPU6050(mux[mux_pos])
    except:
        print("gyro init failed")
    return mpu


def get_readings(mpu, rate):
   return get_position(mpu, 1/rate)[2] / pi * 180


def run_sensor_node():  
    pub = rospy.Publisher('/gyro_data', GyroData, queue_size=1)
    rospy.init_node("GyroNode")

    rospy.loginfo("Initializing Sensors")
    mpu = init_sensors()
    rospy.loginfo("Finished Initializing Sensors")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        reading = get_readings(mpu, 20)
        pub.publish(reading)
        rate.sleep()
        

if __name__ == "__main__":
    try:
        run_sensor_node()
    except rospy.ROSInterruptException:
        pass
