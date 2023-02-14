import time
import board
import adafruit_mpu6050
import adafruit_tca9548a
import numpy as np

delta_x = 0
delta_y = 0
delta_z = 0
thresh_x = 0
thresh_y = 0
thresh_z = 0

CAL_N = 256
T = 0.01

i2c = board.I2C()
mux = adafruit_tca9548a.TCA9548A(i2c)
mux_pos = 6

try:
    mpu = adafruit_mpu6050.MPU6050(mux[mux_pos])

except:
    print("boohoo, mux/mpu failed")

def readNormGyro():
    raw_x, raw_y, raw_z = readRawGyro()
    norm_x = raw_x - delta_x
    norm_y = raw_y - delta_y
    norm_z = raw_z - delta_z

    if thresh_x > abs(norm_x):
        norm_x = 0
    if thresh_y > abs(norm_y):
        norm_y = 0
    if thresh_z > abs(norm_z):
        print("gated")
        norm_z = 0

    return norm_x,norm_y,norm_z

def readRawGyro():
    return mpu.gyro

def calibrateGyro():
    global thresh_x, thresh_y, thresh_z, delta_x, delta_y, delta_z
    sum_x = 0
    sum_y = 0
    sum_z = 0
    var_x = 0
    var_y = 0
    var_z = 0
    for i in range(CAL_N):
        raw_x, raw_y, raw_z = readRawGyro()
        if i > 50:
            sum_x += raw_x
            sum_y += raw_y
            sum_z += raw_z

            var_x += raw_x**2
            var_y += raw_y**2
            var_z += raw_z**2
        time.sleep(T)

    delta_x = sum_x / CAL_N #u
    delta_y = sum_y / CAL_N
    delta_z = sum_z / CAL_N

    thresh_x = (var_x/CAL_N)**0.5 - delta_x**2
    thresh_y = (var_y/CAL_N)**0.5 - delta_y**2
    thresh_z = (var_z/CAL_N)**0.5 - delta_z**2
    print(f"thresh_z : {thresh_z}")



calibrateGyro()
pitch = 0
roll = 0
yaw = 0

while True:
    x, y, z = readNormGyro()
    pitch = pitch + y * T
    roll = roll + x * T    
    yaw = yaw + z * T
    
    #print(f"Acceleration: X:{x:.2f}, Y: {y:.2f}, Z: {z:.2f} m/s^2")
    #print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s"%(mpu.gyro))
    #print("Temperature: %.2f C"%mpu.temperature)
    
    print(f"pitch: {pitch:.5f} roll: {roll:.5f} yaw: {yaw/0.00872664625:.5f}", end='\r')
    time.sleep(T)
