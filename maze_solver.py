from XRPLib.defaults import *

#from lib.XRPLib.pid import PID
#from lib.XRPLib.timeout import Timeout

from VL53L0X import *
from machine import I2C,Pin
import time
import math

from twin_potentiometers import *
from maze_profile_agent import *

print("all modules imported")
board.led_blink(16)

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

angle_ku = 0.2*0.25
angle_tu = 1/(600/2)
angle_kp = 0.8*angle_ku
angle_kd = 0.1 * angle_ku * angle_tu

agent = MazeProfileAgent(maze_motor_left, maze_motor_right, forward_kp, forward_kd, angle_kp, angle_kd)

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

# Define motion profiles: [distance (mm), max speed, start speed, end speed, acceleration]
profiles = [
    #[mm, m/s, m/s, m/s, m/s^2],[deg, deg/s, deg/s, deg/s, deg/s^2]
    #[[182*4, 200, 100, 0, 800],[90, 90, 90, 0, 0.25*90]], 
    [[0, 0, 0, 0, 800],[90, 800, 200, 0, 4000]], 
    #[[200, 800, 200, 0, 3200],[0, 0, 0, 0, 10000000]],

]

#agent.hold_stability(True, True)


# Execute the motion profiles
#maze_motor_left.set_effort(0.3)
#maze_motor_right.set_effort(0.3)
agent.execute_profiles(profiles)


while True:
    pass
    #maze_motor_left.set_effort((get_PotADC1() * 2) -1)
    #maze_motor_right.set_effort((get_PotADC2() * 2) -1)
    #print(agent.get_avg_position_mm(), agent.get_avg_angle_deg())
#print(tof0.ping(), rangefinder.distance(), tof1.ping())
    
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