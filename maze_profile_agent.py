from XRPLib.defaults import *

from lib.XRPLib.pid import PID
from lib.XRPLib.timeout import Timeout

import time
import math

class MazeProfileAgent:
    
    def __init__(self, maze_motor_left, maze_motor_right, max_effort):
        self.left_motor = maze_motor_left
        self.right_motor = maze_motor_right
        self.forward_pid = PID(
            kp=0.001,
            ki=0.0,
            kd=0.0,
            min_output=0.0,
            max_output=max_effort,
        )
        self.rotation_pid = PID(1.0, 0.0, 0.1)  # Adjust PID values as needed

    def execute_profiles(self, profiles):
        for profile in profiles:
                self.execute_profile(profile)

    def execute_profile(self, profile):
        # mm, mm/s, mm/s, mm/s, mm/s^2
        #acceleration is a scaler here
        target_distance, max_speed, start_speed, end_speed, max_acceleration = profile
        
        #start_speed cannot be zero, system needs a kick. this fix below does not work great
        #if start_speed == 0:
        #    start_speed = math.sqrt(2 * max_acceleration * dt)  # Small initial push

        #solve for and set peak_speed, the highest speed the motion will ever reach. can be lower than max_speed
        vp_squared = (start_speed**2 + end_speed**2)/2 + (max_acceleration * target_distance)
        peak_speed = math.sqrt(vp_squared) if vp_squared >= 0 else 0
        if peak_speed > max_speed:
            peak_speed = max_speed

        deceleration_distance = ((peak_speed**2) - (end_speed**2)) / (2 * max_acceleration)
        acceleration_distance = ((peak_speed**2) - (start_speed**2)) / (2 * max_acceleration)

        #can be negative for short paths but thats no prob
        max_speed_distance = target_distance - acceleration_distance - deceleration_distance


        start_position = (self.left_motor.get_position_counts() + self.right_motor.get_position_counts()) / 2
        
        previous_time = time.ticks_ms()
        previous_position = start_position


        while True:

            current_position = (self.left_motor.get_position_counts() + self.right_motor.get_position_counts()) / 2

            current_distance = current_position - start_position
            remaining_distance = target_distance - current_distance
            
            if remaining_distance > (max_speed_distance + deceleration_distance):
                v_squared = start_speed**2 + 2 * max_acceleration * current_distance
                target_velocity = math.sqrt(v_squared)
                
            elif remaining_distance > deceleration_distance:
                target_velocity = max_speed
                
            elif remaining_distance > 0:
                deceleration_progress_distance = deceleration_distance - remaining_distance
                v_squared = peak_speed**2 - 2 * max_acceleration * deceleration_progress_distance
                target_velocity = math.sqrt(v_squared) if v_squared >= 0 else 0

            else:
                target_velocity = end_speed

            current_time = time.ticks_ms()
            delta_t = time.ticks_diff(current_time, previous_time) / 1000

            target_position = previous_position + (target_velocity * delta_t) 
            error_position = target_position - current_position
            
            # cache values for next update
            previous_time = current_time
            previous_position = current_position


            effort = self.forward_pid.update(error_position)

            self.left_motor.set_effort(effort)
            self.right_motor.set_effort(effort)

            

            





