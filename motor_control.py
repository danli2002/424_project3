import board
import busio
import adafruit_mcp4728
import time

# values for controlling the steering potentiometer
# steer values for now are set to the "extremes", i.e.
# 0 is the leftmost the car can turn, 65535 is the rightmost etc
MAX_STEER_RIGHT = 65535
MAX_STEER_LEFT = 0
STEER_RESET = 32500

# experimentally determined value for a controlled acceleration
FORWARD = 65535
# 2.1V = neutral
# midpt of 65535 is 32767
STOP = 32767

# might not need this
REVERSE = 0

class MotorControl:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA) # uses board.SCL and board.SDA
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
        #  use for MCP4728 variant
        #  Note: we overrode the address because apparently the board was connected to 0x64
        self.mcp4728 = adafruit_mcp4728.MCP4728(self.i2c, 0x64)

        # initializing the channels for steering and throttle
        self.throttle = self.mcp4728.channel_b 
        self.steering = self.mcp4728.channel_a

        self.stop()
        self.steer_neutral()

        self.mcp4728.save_settings() 
    
    def steer_right(self, value = 1.0):
        print(f'Value: {int(STEER_RESET * value) + STEER_RESET}')
        self.steering.value = min(int(STEER_RESET * value) + STEER_RESET, 65535)

    def steer_left(self, value = 1.0):
        print(f'Value: {STEER_RESET - int(STEER_RESET * value)}')
        self.steering.value = max(0,STEER_RESET - int(STEER_RESET * value))

    def steer_neutral(self):
        self.steering.value = STEER_RESET
    
    def go_forward(self, val = 1.0):
        val = max(min(1.0, val), 0)
        self.throttle.value = int(FORWARD * val/2 + STOP)
    
    def stop(self):
        self.throttle.value = STOP
