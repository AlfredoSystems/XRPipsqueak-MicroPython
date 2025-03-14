from XRPLib.defaults import *

#from lib.XRPLib.pid import PID
#from lib.XRPLib.timeout import Timeout

from VL53L0X import *
from machine import I2C,Pin
import time
import math

from maze_utils import *

maze = MazeAgent()

i2c0 = I2C(0, scl=Pin(21), sda=Pin(20), freq=50000)
i2c1 = I2C(1, scl=Pin(19), sda=Pin(18), freq=50000)

tof0 = VL53L0X(i2c0)
tof1 = VL53L0X(i2c1)

tof0.set_measurement_timing_budget(40000)
tof1.set_measurement_timing_budget(40000)

tof0.set_Vcsel_pulse_period(tof0.vcsel_period_type[0], 12)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[0], 12)

tof0.set_Vcsel_pulse_period(tof0.vcsel_period_type[1], 8)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[1], 8)



print("waiting for button")
board.led_blink(4)

# Wait until user command before running
#while not board.is_button_pressed():
#    time.sleep(.01)
# Wait until user to release button before running
#while board.is_button_pressed():
#    time.sleep(.01)
    
#print("starting")
#board.led_off()


#for sides in range(4):
#    maze.straight(182, 0.3, 10) #mm, effort, seconds
#    time.sleep(0.05)
#    maze.turn(90, 0.3, 100) #degrees, effort, seconds
#    time.sleep(0.05)

maze.straight(182/2, 0.3, 10)

while True:
    print(tof0.ping(), rangefinder.distance(), tof1.ping())
    
    if(tof0.ping() > 100):
        #forward left forward
        maze.straight(182/2 + 5, 0.3, 10)
        maze.turn(90, 0.3, 10)
        maze.straight(182/2, 0.3, 10)
        
    elif(tof1.ping() > 100):
        #forward right forward
        maze.straight(182/2 + 5, 0.3, 10)
        maze.turn(-90, 0.3, 10)
        maze.straight(182/2, 0.3, 10)
        
    elif(rangefinder.distance() > 20):
        #forward
        maze.straight(182, 0.3, 10)
        
    else:
        #turn around
        maze.turn(180, 0.3, 10)

    time.sleep(0.1)



    