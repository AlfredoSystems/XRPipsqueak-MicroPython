from XRPLib.defaults import *
from lib.XRPLib.imu import IMU

#from lib.XRPLib.pid import PID
#from lib.XRPLib.timeout import Timeout

from VL53L0X import *
from machine import I2C,Pin
import time
import math

from maze_profile_agent import *
from maze_flood_fill import *

print("all modules imported")
board.led_blink(16)

i2c0 = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)
time.sleep(0.1)
i2c1 = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
time.sleep(0.1)

imu = IMU(i2c1)
imu.calibrate()

#ToF: 50 -> too close, 70 -> wall perfect, 90 -> wall max, 230 -> open
#Sonar: 2 - too close, 3 - wall perfect, 13 -> wall detected from cell entrance, 30, no wall
maze_motor_left = EncodedMotor(
                    SinglePWMMotor(2, 3),
                    Encoder(2, 1, 0, flip_dir=True)
                )
maze_motor_right = EncodedMotor(
                    SinglePWMMotor(10, 11, flip_dir=True),
                    Encoder(3, 9, 8, flip_dir=True)
                )
                
forward_ku = 0.28*0.25
forward_tu = 1/(480/2)
forward_kp = 0.8*forward_ku
forward_kd =  0.1 * forward_ku * forward_tu

angle_ku = 0.2*0.25*0.75
angle_tu = 1/(600/2)
angle_kp = 0.8*angle_ku
angle_kd = 0.1 * angle_ku * angle_tu

agent = MazeProfileAgent(maze_motor_left, maze_motor_right, forward_kp, forward_kd, angle_kp, angle_kd, imu)
maze = MazeAgent()

tof0 = VL53L0X(i2c0)
tof1 = VL53L0X(i2c1)
time.sleep(0.1)
tof0.set_measurement_timing_budget(40000)
tof1.set_measurement_timing_budget(40000)
tof0.set_Vcsel_pulse_period(tof0.vcsel_period_type[0], 12)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[0], 12)
tof0.set_Vcsel_pulse_period(tof0.vcsel_period_type[1], 8)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[1], 8)
time.sleep(0.1)

board.led_blink(4)

# Define motion profiles: [distance (mm), max speed, start speed, end_speed, acceleration]
straight_h = [[182/2, 800, 400, 50, 3200, 1],[0, 1600, 0, 0, 2*3200, 1]]
#straight_1 = [[182, 800, 400, 100, 3200, 1],[0, 1600, 0, 0, 2*3200, 1]]

#turn_left = [[0, 800, 0, 0, 3200, 1],[90, 1600, 400, 200, 2*6400, -1]]
#turn_right = [[0, 800, 0, 0, 3200, 1],[90, 1600, 400, 200, 2*6400, 1]]


agent.execute_profiles([straight_h])
time.sleep(0.1)

while True:

    center_dist = rangefinder.distance()
    left_dist = tof0.ping()
    right_dist = tof1.ping()
    print(left_dist, center_dist, right_dist)

    if(center_dist < 20):
        maze.add_forward_wall()

    if(left_dist < 150):
        maze.add_left_wall()

    if(right_dist < 150):
        maze.add_right_wall()

    if(center_dist < 20 and left_dist < 150 and right_dist < 150):
        agent.move_turn_around(center_dist, left_dist, right_dist)
        maze.turn_orientation_right()
        maze.turn_orientation_right()
    
    elif(center_dist < 20 and left_dist < 150):
        agent.move_right(center_dist, left_dist, right_dist)
        maze.turn_orientation_right()
        maze.move_forward()

    elif(left_dist < 150):
        agent.move_forward(center_dist, left_dist, right_dist)    
        maze.move_forward()

    else:
        agent.move_left(center_dist, left_dist, right_dist)
        maze.turn_orientation_left()
        maze.move_forward()

    time.sleep(0.1)