from XRPLib.defaults import *

from lib.XRPLib.pid import PID
from lib.XRPLib.timeout import Timeout

import time
import math

class VelocityTargetGenerator:
    def __init__(self, profile):
        target_distance, max_speed, start_speed, end_speed, max_acceleration = profile
        self.target_distance = target_distance
        self.max_speed = max_speed
        self.start_speed = start_speed
        self.end_speed = end_speed
        self.max_acceleration = max_acceleration
        
        self.peak_speed = self.calculate_peak_speed()
        self.deceleration_distance = self.calculate_deceleration_distance()
        self.acceleration_distance = self.calculate_acceleration_distance()
        self.max_speed_distance = self.target_distance - self.acceleration_distance - self.deceleration_distance
    
    def calculate_peak_speed(self):
        vp_squared = (self.start_speed**2 + self.end_speed**2) / 2 + (self.max_acceleration * self.target_distance)
        peak_speed = math.sqrt(vp_squared) if vp_squared >= 0 else 0
        return min(peak_speed, self.max_speed)
    
    def calculate_deceleration_distance(self):
        return ((self.peak_speed**2) - (self.end_speed**2)) / (2 * self.max_acceleration)
    
    def calculate_acceleration_distance(self):
        return ((self.peak_speed**2) - (self.start_speed**2)) / (2 * self.max_acceleration)
    
    def get_target_velocity(self, current_distance):
        remaining_distance = self.target_distance - current_distance
        
        if remaining_distance > (self.max_speed_distance + self.deceleration_distance):
            v_squared = self.start_speed**2 + 2 * self.max_acceleration * current_distance
            return math.sqrt(v_squared) if v_squared >= 0 else 0, "accel"
        
        elif remaining_distance > self.deceleration_distance:
            return self.max_speed, "maxim"
        
        elif remaining_distance > 0:
            deceleration_progress_distance = self.deceleration_distance - remaining_distance
            v_squared = self.peak_speed**2 - 2 * self.max_acceleration * deceleration_progress_distance
            return math.sqrt(v_squared) if v_squared >= 0 else 0, "decel"
        
        else:
            return self.end_speed, "final"

class MazeProfileAgent:
    
    def __init__(self, maze_motor_left, maze_motor_right, forward_kp, forward_kd, angle_kp, angle_kd):
        self.left_motor = maze_motor_left
        self.right_motor = maze_motor_right
        self.WHEEL_DIAMETER_MM = 33.86 #32mm irl
        self.CENTER_TO_CENTER_MM = 82.1435092593 # 78mm irl
        self.TICKS_PER_REV = 800
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER_MM
        
        self.forward_pid = PID(
            kp=forward_kp,
            ki=0.0,
            kd=forward_kd,
            min_output=0.0,
            max_output=1.0,
        )
        self.angle_pid = PID(
            kp=angle_kp,
            ki=0.0,
            kd=angle_kd,
            min_output=0.0,
            max_output=1.0,
        )

    def execute_profiles(self, profiles):
        for profile in profiles:
            
            self.execute_profile(profile[0], profile[1])
                
    def get_avg_position_mm(self):
        ticks_position = (self.left_motor.get_position_counts() + self.right_motor.get_position_counts()) / 2
        mm_per_tick = self.WHEEL_CIRCUMFERENCE / self.TICKS_PER_REV
        return ticks_position * mm_per_tick
    
    def get_avg_angle_deg(self):
        ticks_angle = (self.left_motor.get_position_counts() - self.right_motor.get_position_counts()) / self.CENTER_TO_CENTER_MM
        deg_per_tick = (180/math.pi) * self.WHEEL_CIRCUMFERENCE / self.TICKS_PER_REV 
        return ticks_angle * deg_per_tick
        
    def execute_profile(self, mm_profile, deg_profile):
        target_distance, max_speed, start_speed, end_speed, max_acceleration = mm_profile
        target_angle, max_omega, start_omega, end_omega, max_alpha = deg_profile

        speed_generator = VelocityTargetGenerator(mm_profile)
        omega_generator = VelocityTargetGenerator(deg_profile)

        start_position = self.get_avg_position_mm()
        start_angle = self.get_avg_position_mm()
        
        previous_time = time.ticks_ms()
        
        previous_position = start_position
        previous_angle = start_angle

        while True:
            current_position = self.get_avg_position_mm()
            current_position_delta = current_position - start_position
            target_velocity, velocity_state = speed_generator.get_target_velocity(current_position_delta)
            
            current_angle = self.get_avg_angle_deg()
            current_angle_delta = current_angle - start_angle
            target_omega, omega_state = omega_generator.get_target_velocity(current_angle_delta)

            current_time = time.ticks_ms()
            delta_t = time.ticks_diff(current_time, previous_time) / 1000
            
            error_position =  previous_position + (target_velocity * delta_t) - current_position
            error_angle = previous_angle + (target_omega * delta_t) - current_angle
            
            previous_time = current_time
            previous_position = current_position
            previous_angle = current_angle
            
            forward_effort = self.forward_pid.update(error_position)
            angle_effort = self.angle_pid.update(error_angle)
            #print(f"{velocity_state:2} {forward_effort:10.2f} {current_position:10.2f} {error_position:10.2f}  {target_velocity:10.2f}")
            print(f"{omega_state:2} {angle_effort:10.2f} {current_angle:10.2f} {error_angle:10.2f}  {target_omega:10.2f}")

            self.left_motor.set_effort(forward_effort + angle_effort)
            self.right_motor.set_effort(forward_effort - angle_effort)
            
            #if current_distance >= target_distance and current_angle >= target_angle:
            #    self.left_motor.set_effort(0)
            #    self.right_motor.set_effort(0)
            #    break
            
            time.sleep(.01)

    def hold_stability(self, hold_position=True, hold_angle=True):
    
        start_position = self.get_avg_position_mm()
        start_angle = self.get_avg_angle_deg()
        previous_time = time.ticks_ms()
    
        while True:
            current_position = self.get_avg_position_mm()
            current_angle = self.get_avg_angle_deg()
            
            error_position = start_position - current_position if hold_position else 0
            error_angle = start_angle - current_angle if hold_angle else 0
    
            current_time = time.ticks_ms()
            delta_t = time.ticks_diff(current_time, previous_time) / 1000
            previous_time = current_time
    
            forward_effort = self.forward_pid.update(error_position) if hold_position else 0
            angle_effort = self.angle_pid.update(error_angle) if hold_angle else 0
    
            self.left_motor.set_effort(forward_effort + angle_effort)
            self.right_motor.set_effort(forward_effort - angle_effort)
            
            print(f"Position Error: {error_position:.2f} mm | Angle Error: {error_angle:.2f}� | "
                  f"Forward Effort: {forward_effort:.2f} | Angle Effort: {angle_effort:.2f}")
            
            time.sleep(0.01)
