import time
import board
import adafruit_mpu6050
import adafruit_tca9548a

i2c = board.I2C()
mux = adafruit_tca9548a.TCA9548A(i2c)
mux_pos = 6
try:
    mpu = adafruit_mpu6050.MPU6050(mux[mux_pos])

except:
    print("boohoo, mux/mpu failed")

while True:
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2"%(mpu.acceleration))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s"%(mpu.gyro))
    print("Temperature: %.2f C"%mpu.temperature)
    print("")
    time.sleep(0.01)
