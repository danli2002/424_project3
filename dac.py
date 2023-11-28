'''
Code that performs basic acceleration and steering tests on  RC car
using RPi 4 and Adafruit DAC

Daniel Li, Daniel Choi, Desmond Roberts, Akshay Shyam, Samatar Dalmar
11/28/2023
'''
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import board
import adafruit_mcp4728
import time

# values for controlling the steering potentiometer
# steer values for now are set to the "extremes", i.e.
# 0 is the leftmost the car can turn, 65535 is the rightmost etc
STEER_RIGHT = 65535
STEER_LEFT = 0
STEER_RESET = 32500

# experimentally determined value for a controlled acceleration
FORWARD = 35000
# 2.1V = neutral
# midpt of 65535 is 32767
STOP = 32767

# might not need this
REVERSE = 0

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
#  use for MCP4728 variant
#  Note: we overrode the address because apparently the board was connected to 0x64
mcp4728 = adafruit_mcp4728.MCP4728(i2c, 0x64)

# initializing the channels for steering and throttle
throttle = mcp4728.channel_b 
steering = mcp4728.channel_a

# test function for steering the car left and right and back to neutral
def test_steer():
    # reset steer to neutral
    steering.value = STEER_RESET
    time.sleep(1)

    # steer left
    steering.value = STEER_LEFT
    time.sleep(1)
    print("left steer done")
   
    steering.value = STEER_RESET
    time.sleep(1)
    print("reset")

    # steer right
    steering.value = STEER_RIGHT
    print("right steer done")
    time.sleep(1)
    steering.value = STEER_RESET

# test function to accelerate for 3 seconds and then stop
def test_throttle():
    # stop the wheels if they are moving
    throttle.value = STOP
    time.sleep(1)
    
    print("starting")
    # go forward
    throttle.value = FORWARD
    time.sleep(3)
    print("stopping")
    # stopping at end
    throttle.value = STOP

# running the tests
if __name__ == "__main__":
    test_steer()
    time.sleep(1)
    test_throttle()
