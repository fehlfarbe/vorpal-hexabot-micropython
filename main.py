# main.py -- put your code here!
from machine import Pin, I2C
import pycom
import time
from vorpal import VorpalHexabot

pycom.heartbeat(False)
pycom.rgbled(0x001100)

# configure the I2C bus
i2c = I2C(baudrate=400000)
print(i2c.scan()) # returns list of slave addresses
robot = VorpalHexabot(i2c)

# main loop
print("Starting main loop")
robot.beep()
while True:
    print("waiting for cmd")
    time.sleep_ms(100)

print("Exited main loop")
