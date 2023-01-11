#!/usr/bin/python3
from sdp.msg import ToFData
import rospy
import adafruit_vl53l4cd
import adafruit_tca9548a
from board import SCL, SDA
import busio

INTER_MEASUREMENT = 0
TIMING_BUDGET = 200

RANGING = False
tof = None

def init_sensors():
    i2c = busio.I2C(SCL, SDA)
    mux = adafruit_tca9548a.TCA9548A(i2c)
    #mux_pos_list = [4,3,2,1,0,5]
    mux_pos_list = [5]
    tof_sensors = []
    for mux_pos_index in mux_pos_list:
        print(f"Trying to init {mux_pos_index}")
        try:
            tof_sensors.append(adafruit_vl53l4cd.VL53L4CD(mux[mux_pos_index]))
            print(f"sensor initialized: "
                    f"{mux_pos_index + 1 if mux_pos_index < 5 else 0}" 
                    f" sensors remaining")
        except:
            print(f"Oops, {mux_pos_index}")
    return tof_sensors

def set_sensor_parameters(tof_sensors, inter_measurement, timing_budget):
    for sensor in tof_sensors:
        sensor.inter_measurement = inter_measurement
        sensor.timing_budget = timing_budget

def start_ranging(tof):
    for sensor in tof:
        sensor.start_ranging()

def stop_ranging(tof):
    for sensor in tof:
        sensor.stop_ranging()

def get_readings(tof):
    data = [0] * len(tof)
    for i in range(len(tof)):
        tof[i].clear_interrupt()
        tof[i].distance
    for i, sensor in enumerate(tof):
        while not sensor.data_ready:
            continue
        if sensor.data_status == 0:
            data[i] = sensor.distance
            sensor.clear_interrupt()
        else:
            data[i] = -1
    return data
    """
    for i in range(3):
        for i, sensor in enumerate(tof):
            while not sensor.data_ready:
                continue
            data[i] = 0.1 * data[i] + 0.9 * sensor.distance
            sensor.clear_interrupt()
    """
    return data

def run_sensor_node():  
    global tof

    pub = rospy.Publisher('/tof_data', ToFData, queue_size=1)
    rospy.init_node("ToFNode")

    rospy.loginfo("Initializing Sensors")
    tof = init_sensors()
    rospy.loginfo("Finished Initializing Sensors")
    start_ranging(tof)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        reading = get_readings(tof)
        reading = [reading[0], 0., 0., 0., 0., 0.]
        pub.publish(reading)
        rate.sleep()
        

if __name__ == "__main__":
    try:
        run_sensor_node()
    except rospy.ROSInterruptException:
        pass
