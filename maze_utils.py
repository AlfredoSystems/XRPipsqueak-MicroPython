from XRPLib.defaults import *

from lib.XRPLib.pid import PID
from lib.XRPLib.timeout import Timeout

import time
import math

class MazeAgent:
    
    def __init__(self):
        pass
        
    def straight(self, distance, max_effort, timeout):
        
        distance *= 1000/133 #1000 ticks = 133 mm
    
        time_out = Timeout(timeout)
        starting_left = left_motor.get_position_counts()
        starting_right = right_motor.get_position_counts()
    
        main_controller = PID(
            kp=0.01,
            ki=0.0,
            kd=0.0,
            min_output=0.0,
            max_output=max_effort,
            max_integral=0,
            tolerance=2 * 1000/133,
            tolerance_count=4,
        )
        
        secondary_controller = PID(
            kp=0.02,
            kd=0.02,
            max_output = max_effort,
            )
        # Record initial heading
        initial_heading = imu.get_yaw()
    
        while True:
            # Calculate the distance traveled
            left_delta = left_motor.get_position_counts() - starting_left
            right_delta = right_motor.get_position_counts() - starting_right
            dist_traveled = (left_delta + right_delta) / 2
            
    
            # PID for distance
            distance_error = distance - dist_traveled
            effort = main_controller.update(distance_error)
    
            if main_controller.is_done() or time_out.is_done():
                break
    
            # Calculate heading correction
            current_heading = imu.get_yaw()
            heading_correction = secondary_controller.update(initial_heading - current_heading)
            heading_correction = 0
            
            drivetrain.set_effort(-(effort - heading_correction), -(effort + heading_correction))
    
            time.sleep(0.01)
    
        drivetrain.stop()
    
        return not time_out.is_done()
    
    def turn(self, turn_degrees, max_effort, timeout, _kp = 0, _kd = 0):
    
        time_out = Timeout(timeout)
        starting_left = left_motor.get_position_counts()
        starting_right = right_motor.get_position_counts()
    
        Ku = 0.06
        Tu = 0.26
        
        main_controller = PID(
            kp = 0.6*Ku,
            ki = 2*0.6*Ku/Tu,
            kd = 0.6*Ku*Tu/8,
            min_output = 0,
            max_output = max_effort,
            max_integral = 0,
            tolerance = 1,
            tolerance_count = 3
        )
    
        secondary_controller = PID(
            kp = 0.06,
            max_output = max_effort,
            )
    
        turn_degrees += imu.get_yaw()
    
        while True:
            
            # calculate encoder correction to minimize drift
            left_delta = left_motor.get_position_counts() - starting_left
            right_delta = right_motor.get_position_counts() - starting_right
            encoder_correction = secondary_controller.update(left_delta + right_delta)
            encoder_correction = 0
    
    
            turn_error = turn_degrees - imu.get_yaw()
            #turn_error = turn_degrees - ((right_delta-left_delta)/2)*360/(self.track_width*math.pi)
            
            print(turn_error)
            # Pass the turn error to the main controller to get a turn speed
            turn_speed = main_controller.update(turn_error)
    
            # exit if timeout or tolerance reached
            if main_controller.is_done() or time_out.is_done():
                break
    
            drivetrain.set_effort((turn_speed - encoder_correction), -(turn_speed + encoder_correction))
    
            time.sleep(0.01)
    
        drivetrain.stop()
    
        return not time_out.is_done()