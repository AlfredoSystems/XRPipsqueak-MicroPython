from XRPLib.defaults import *
from lib.XRPLib.imu import IMU

#from lib.XRPLib.pid import PID
#from lib.XRPLib.timeout import Timeout

from VL53L0X import *
from machine import I2C,Pin
import time
import math

from maze_profile_agent import *

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


straight_h = [[182/2, 800, 400, 50, 3200, 1],[0, 1600, 0, 0, 2*3200, 1]]
straight_1 = [[182, 800, 400, 100, 3200, 1],[0, 1600, 0, 0, 2*3200, 1]]

turn_left = [[0, 800, 0, 0, 3200, 1],[90, 1600, 400, 200, 2*6400, -1]]
turn_right = [[0, 800, 0, 0, 3200, 1],[90, 1600, 400, 200, 2*6400, 1]]
# Define motion profiles: [distance (mm), max speed, start speed, end_speed, acceleration]

#agent.hold_stability(True, True)

#maze_motor_left.set_effort(0)
#maze_motor_right.set_effort(0)

agent.execute_profiles([straight_h])
time.sleep(0.1)

while True:
    #maze_motor_left.set_effort((get_PotADC1() * 2) -1)
    #maze_motor_right.set_effort((get_PotADC2() * 2) -1)
    #print(agent.get_avg_position_mm(), agent.get_avg_angle_deg())
    center_dist = rangefinder.distance()
    left_dist = tof0.ping()
    right_dist = tof1.ping()
    print(left_dist, center_dist, right_dist)

    if(left_dist > 150):
        #forward left forward

        if(center_dist < 20):
            dist_offset = (center_dist - 13) * 0.1 #cm to mm
        
        if(right_dist < 150):
            angle_offset = (right_dist - 70) * (5/20) #20 units = 5 deg correction?
        
        agent.execute_profiles([
            [[182/2 + dist_offset, 800, 400, 50, 3200, 1],[0 + angle_offset, 1600, 0, 0, 2*3200, 1]],
            turn_left,
            straight_h,
            ])

    elif(center_dist > 20):
        #forward

        if(left_dist < 150):
            angle_offset_left = (left_dist - 70) * (5/20) #20 units = 5 deg correction?
        
        if(right_dist < 150):
            angle_offset_right = (left_dist - 70) * (5/20) #20 units = 5 deg correction?
    
        agent.execute_profiles([
            [[182/2, 800, 400, 50, 3200, 1],[0 + angle_offset_right - angle_offset_left, 1600, 0, 0, 2*3200, 1]],
            ])     
    
    elif(right_dist > 150):

        if(center_dist < 20):
            dist_offset = (center_dist - 13) * 0.1 #cm to mm
        
        if(left_dist < 150):
            angle_offset = (left_dist - 70) * (5/20) #20 units = 5 deg correction?

        #forward right forward
        agent.execute_profiles([
            [[182/2 + dist_offset, 800, 400, 50, 3200, 1],[0 + angle_offset, 1600, 0, 0, 2*3200, 1]],
            turn_right,
            straight_h,
            ])
   
    else:
        #turn around
        agent.execute_profiles([
            [[0, 800, 0, 0, 3200, 1],[180, 1600, 400, 200, 2*6400, 1]],
            ])    

    time.sleep(0.1)