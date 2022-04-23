import time
#from adafruit_servokit import ServoKit
from board import SCL_1, SDA_1
import busio
from adafruit_pca9685 import PCA9685

#from adafruit_motor import servo
i2c = busio.I2C(SCL_1, SDA_1)
pca = PCA9685(i2c, address = 0x40)
# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
#kit = ServoKit(channels=16)

#kit.servo[3].angle = 180
#time.sleep(1)
#time.sleep(1)
#kit.servo[3].angle = 0
print("starting prgram...")
print("Input desired servo channel(DEFAULT=15): ")
ch=int(input() or "15")
pca.frequency = 50
#for i in range(0, 16):
#    ch = pca.channels[i]
#    ch.duty_cycle = 0x7FFF
#pca.channels[15].duty_cycle = 32767
#ch = pca.channels[3]
#ch.duty_cycle = 32767
#time.sleep(2)
#ch.duty_cycle = 0
while(1):
    dc = input("Input Duty Cycle: ")
    dc = int(dc)
    if(dc >= 0 and dc <= 100):
        val = 65535 * (dc/100)
        val = int(val)
        print(val)
        pca.channels[ch].duty_cycle = val
    else:
        print("Invalid input, try again...")
