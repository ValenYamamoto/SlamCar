#!/usr/bin/python3
from sdp.srv import (
    DistanceData, 
    DistanceDataResponse, 
    ConfigSensors,
    ConfigSensorsResponse,
    StartSensors,
    StartSensorsResponse
)
import rospy
import adafruit_vl53l4cd
import adafruit_tca9548a
from board import SCL, SDA
import busio

INTER_MEASUREMENT = 0
TIMING_BUDGET = 200

RANGING = False
tof = None

def handle_reading_request(req):
    global tof
    if tof is None:
        return DistanceDataResponse([0, 0, 0, 0, 0, 0], 1)
    if RANGING:
        readings = get_readings(tof)
        return DistanceDataResponse(readings, 0)
    return DistanceDataResponse([0, 0, 0, 0, 0, 0], 0)

def handle_start_request(req):
    global RANGING, tof
    if req.start:
        start_ranging(tof)
        RANGING = True
    else:
        stop_ranging(tof)
        RANGING = False
    return StartSensorsResponse(0)

def handle_config_request(req):
    global tof
    set_sensor_parameters(tof, reg.inter_measurement, req.timing_budget)
    return ConfigSensorsResponse(0)

def init_sensors():
    i2c = busio.I2C(SCL, SDA)
    mux = adafruit_tca9548a.TCA9548A(i2c)
    mux_pos_list = [4,3,2,1,0,5]
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
        data[i] = sensor.distance
        sensor.clear_interrupt()
        #data[i] = sensor.distance
    return data
    for i in range(3):
        for i, sensor in enumerate(tof):
            while not sensor.data_ready:
                continue
            data[i] = 0.1 * data[i] + 0.9 * sensor.distance
            sensor.clear_interrupt()
    return data

def run_sensor_node():  
    global tof

    rospy.init_node("ToFNode")

    rospy.loginfo("Initializing Sensors")
    tof = init_sensors()
    rospy.loginfo("Finished Initializing Sensors")
        
    reading = rospy.Service('tof_readings', DistanceData, handle_reading_request)
    start = rospy.Service('tof_start', StartSensors, handle_start_request)
    config = rospy.Service('tof_config', ConfigSensors, handle_config_request)

    rospy.spin()

if __name__ == "__main__":
    run_sensor_node()
