# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import board
import adafruit_mcp4728
import time

MCP4728_DEFAULT_ADDRESS = 0x60

'''
DO NOT USE THESE VALUES AS WE HAVEN'T DETERMINED
THE RIGHT VOLTAGE VALUES
'''

STEER_RIGHT = 65535
STEER_LEFT = 0
STEER_RESET = 2.1 / 3.3 * STEER_RIGHT



# placeholder value for the max acceleration voltage
FORWARD = 65535
# 2.1V = neutral
STOP = 2.1 / 3.3 * FORWARD
REVERSE = 0

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
#  use for MCP4728 variant
mcp4728 = adafruit_mcp4728.MCP4728(i2c, adafruit_mcp4728.MCP4728_DEFAULT_ADDRESS)
#  use for MCP4728A4 variant
#  mcp4728 = adafruit_mcp4728.MCP4728(i2c, adafruit_mcp4728.MCP4728A4_DEFAULT_ADDRESS)

throttle = mcp4728.channel_a 
steering = mcp4728.channel_b

def test_steer():
    # reset steer to neutral
    steering.value = STEER_RESET
    time.sleep(1)

    # steer left
    steering.value = STEER_LEFT
    time.sleep(1)

    # steer right
    steering.value = STEER_RIGHT

def test_throttle():
    # stop the wheels if they are moving
    throttle.value = STOP
    time.sleep(1)

    # go forward
    throtte.value = FORWARD
    time.sleep(1)

if __name__ == "__main__":
    test_steer()
    time.sleep(3)
    test_throttle