from XRPLib.defaults import *

#from lib.XRPLib.pid import PID
#from lib.XRPLib.timeout import Timeout

from VL53L0X import *
from machine import I2C,Pin
import time
import math

from maze_profile_agent import *

#ToF: 50 -> too close, 70 -> wall perfect, 90 -> wall max, 230 -> open
#Sonar: 2 - too close, 3 - wall perfect, 13 -> wall detected from cell entrance, 30, no wall
maze_motor_left = EncodedMotor(
                    Motor(2, 3),
                    Encoder(2, 1, 0, flip_dir=True)
                )
maze_motor_right = EncodedMotor(
                    Motor(10, 11, flip_dir=True),
                    Encoder(3, 9, 8, flip_dir=True)
                )

agent = MazeProfileAgent(maze_motor_left, maze_motor_right)

i2c0 = I2C(0, scl=Pin(21), sda=Pin(20), freq=50000)
i2c1 = I2C(1, scl=Pin(19), sda=Pin(18), freq=50000)
time.sleep(0.1)
tof0 = VL53L0X(i2c0)
tof1 = VL53L0X(i2c1)
time.sleep(0.1)
tof0.set_measurement_timing_budget(40000)
tof1.set_measurement_timing_budget(40000)
time.sleep(0.1)
tof0.set_Vcsel_pulse_period(tof0.vcsel_period_type[0], 12)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[0], 12)
time.sleep(0.1)
tof0.set_Vcsel_pulse_period(tof0.vcsel_period_type[1], 8)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[1], 8)

board.led_blink(4)
# Wait until user command before running
while not board.is_button_pressed():
    time.sleep(.01)
# Wait until user to release button before running
while board.is_button_pressed():
    time.sleep(.01)
board.led_off()

#maze.testEncoders()
#maze.square_drive()

# Define motion profiles: [distance (mm), max speed, end speed, acceleration]
profiles = [
    [182, 200, 0, 0, 600], #drive straight 182mm
    #["line", 182/2, 0.8, 0.5, 0.2],  # Move 200mm, max speed 0.8, end speed 0.5, acceleration 0.2
    #["turn", 90, 182/2, 0.5, 0.0, 0.3],  # Move 100mm, max speed 0.5, stop at the end, acceleration 0.3
    #["line", 182/2, 0.6, 0.2, 0.25]  # Move backward 150mm, max speed 0.6, end speed 0.2, acceleration 0.25
]

# Execute the motion profiles
agent.execute_profiles(profiles)

# maze.straight(182/2, 0.3, 10)

# while True:
#     print(tof0.ping(), rangefinder.distance(), tof1.ping())
    
#     if(tof0.ping() > 100):
#         #forward left forward
#         maze.straight(182/2 + 5, 0.3, 10)
#         maze.turn(90, 0.3, 10)
#         maze.straight(182/2, 0.3, 10)
        
#     elif(tof1.ping() > 100):
#         #forward right forward
#         maze.straight(182/2 + 5, 0.3, 10)
#         maze.turn(-90, 0.3, 10)
#         maze.straight(182/2, 0.3, 10)
        
#     elif(rangefinder.distance() > 20):
#         #forward
#         maze.straight(182, 0.3, 10)
        
#     else:
#         #turn around
#         maze.turn(180, 0.3, 10)

#     time.sleep(0.1)