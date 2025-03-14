from XRPLib.defaults import *

from lib.XRPLib.pid import PID
from lib.XRPLib.timeout import Timeout

import time
import math


maze_motor_left = EncodedMotor(
                    Motor(2, 3),
                    Encoder(2, 0, 1)
                )
maze_motor_right = EncodedMotor(
                    Motor(10, 11, flip_dir=True),
                    Encoder(3, 8, 9)
                )

class MazeAgent:
    
    def __init__(self):
        pass
    
    def get_counts_motor_3(self):
        return -1 * maze_motor_left.get_position_counts()
        
    def get_counts_motor_4(self):
        return -1 * maze_motor_right.get_position_counts()

    def testEncoders(self):
        while True:
            maze_motor_left.set_effort(1)
            maze_motor_right.set_effort(1)
            print(self.get_counts_motor_3(), self.get_counts_motor_4())
        
    def straight(self, distance, max_effort, timeout):
        
        distance *= 1000/133 #1000 ticks = 133 mm
    
        time_out = Timeout(timeout)
        starting_left = self.get_counts_motor_3()
        starting_right = self.get_counts_motor_4()
    
        main_controller = PID(
            kp=0.007,
            ki=0.0,
            kd=0.0,
            min_output=0.0,
            max_output=max_effort,
            max_integral=0,
            tolerance=2 * 1000/133,
            tolerance_count=4,
        )
        
        Ku = 0.06
        Tu = 0.26
        secondary_controller = PID(
            kp = 0.6*Ku,
            ki = 2*0.6*Ku/Tu,
            kd = 0.6*Ku*Tu/8,
            min_output = 0,
            max_output = max_effort,
            max_integral = 0.3,
        )
        
        # Record initial heading
        initial_heading = imu.get_yaw()
    
        while True:
            # Calculate the distance traveled
            left_delta = self.get_counts_motor_3() - starting_left
            right_delta = self.get_counts_motor_4() - starting_right
            dist_traveled = (left_delta + right_delta) / 2
            
    
            # PID for distance
            distance_error = distance - dist_traveled
            effort = main_controller.update(distance_error)
    
            if main_controller.is_done() or time_out.is_done():
                break
    
            # Calculate heading correction
            current_heading = imu.get_yaw()
            heading_correction = secondary_controller.update(initial_heading - current_heading)
            
            maze_motor_left.set_effort(effort - heading_correction/2)
            maze_motor_right.set_effort(effort + heading_correction/2)
    
            time.sleep(0.01)
    
        maze_motor_left.set_effort(0)
        maze_motor_right.set_effort(0)
    
        return not time_out.is_done()
    
    def turn(self, turn_degrees, max_effort, timeout, _kp = 0, _kd = 0):
    
        time_out = Timeout(timeout)
        starting_left = self.get_counts_motor_3()
        starting_right = self.get_counts_motor_4()
    
        Ku = 0.04
        Tu = 0.26
        
        main_controller = PID(
            kp = 0.5*Ku,
            ki = 2*0.6*Ku/Tu,
            kd = 2*0.6*Ku*Tu/8,
            min_output = 0,
            max_output = max_effort,
            max_integral = 0.3,
            tolerance = 1,
            tolerance_count = 3,
        )
    
        secondary_controller = PID(
            kp=0.01,
            ki=0.0,
            kd=0.0,
            min_output=0.0,
            max_output=max_effort,
        )
    
        turn_degrees += imu.get_yaw()
    
        while True:
            
            # calculate encoder correction to minimize drift
            left_delta = self.get_counts_motor_3() - starting_left
            right_delta = self.get_counts_motor_4() - starting_right
            encoder_correction = secondary_controller.update(left_delta + right_delta)
            encoder_correction = 0
    
            turn_error = turn_degrees - imu.get_yaw()
            #turn_error = turn_degrees - ((right_delta-left_delta)/2)*360/(self.track_width*math.pi)
            
            # Pass the turn error to the main controller to get a turn speed
            turn_speed = main_controller.update(turn_error)
    
            # exit if timeout or tolerance reached
            if main_controller.is_done() or time_out.is_done():
                break
    
            maze_motor_left.set_effort( -turn_speed + encoder_correction/2)
            maze_motor_right.set_effort( turn_speed + encoder_correction/2)
            
            time.sleep(0.01)
    
        maze_motor_left.set_effort(0)
        maze_motor_right.set_effort(0)
    
        return not time_out.is_done()
    
    def square_drive(self):
        for sides in range(4):
            maze.straight(182, 0.3, 10) #mm, effort, seconds
            time.sleep(0.05)
            maze.turn(90, 0.3, 100) #degrees, effort, seconds
            time.sleep(0.05)
        while True:
            pass