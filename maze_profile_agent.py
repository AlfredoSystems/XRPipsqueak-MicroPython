from XRPLib.defaults import *

from lib.XRPLib.pid import PID
from lib.XRPLib.timeout import Timeout

import time
import math

def sign(x):
    if x >= 0:
        return 1
    else:
        return -1


class VelocityTargetGenerator:
    def __init__(self, profile):
        target_distance, max_speed, start_speed, end_speed, max_acceleration, flip = profile
        self.target_distance = target_distance
        self.max_speed = max_speed
        self.start_speed = start_speed
        self.end_speed = end_speed
        self.max_acceleration = max_acceleration
        
        vp_squared = (self.start_speed**2 + self.end_speed**2) / 2 + (self.max_acceleration * self.target_distance)
        peak_speed = math.sqrt(vp_squared) if vp_squared >= 0 else 0
        self.peak_speed = min(peak_speed, self.max_speed)
        
        self.deceleration_distance = ((self.peak_speed**2) - (self.end_speed**2)) / (2 * self.max_acceleration)
        self.acceleration_distance = ((self.peak_speed**2) - (self.start_speed**2)) / (2 * self.max_acceleration)
        self.max_speed_distance = self.target_distance - self.acceleration_distance - self.deceleration_distance
    
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
            overshoot_distance = current_distance - self.target_distance
            v_squared = self.end_speed**2 - 2 * self.max_acceleration * overshoot_distance
            direction = sign(v_squared)
            return math.sqrt(abs(v_squared)) * direction, "tofar"


class MazeProfileAgent:
    
    def __init__(self, maze_motor_left, maze_motor_right, forward_kp, forward_kd, angle_kp, angle_kd, imu):
        self.left_motor = maze_motor_left
        self.right_motor = maze_motor_right
        self.WHEEL_DIAMETER_MM = 33.86 * (182*3)/(182*3 + 6) #32mm irl, if not going far enough make bigger
        self.CENTER_TO_CENTER_MM = 86 # 78mm irl, if not spinning enough, make bigger
        self.TICKS_PER_REV = 800
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER_MM
        self.imu = imu
        
        self.flip_direction = False
        self.flip_angle = False
        
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
        return self.flip_direction * ticks_position * mm_per_tick
    
    def get_avg_angle_deg(self):
        #ticks_angle = (self.left_motor.get_position_counts() - self.right_motor.get_position_counts()) / self.CENTER_TO_CENTER_MM
        #deg_per_tick = (180/math.pi) * self.WHEEL_CIRCUMFERENCE / self.TICKS_PER_REV 
        #return self.flip_angle * ticks_angle * deg_per_tick
        return -self.imu.get_yaw() * self.flip_angle

        
    def execute_profile(self, mm_profile, deg_profile):

        target_distance, max_speed, start_speed, end_speed, max_acceleration, flip_direction = mm_profile
        target_angle, max_omega, start_omega, end_omega, max_alpha, flip_angle = deg_profile
        
        print(f"new profile: {target_distance:8.2f} {target_angle:8.2f}")        
        
        self.flip_direction = flip_direction
        self.flip_angle = flip_angle
        
        speed_generator = VelocityTargetGenerator(mm_profile)
        omega_generator = VelocityTargetGenerator(deg_profile)
        
        start_position = self.get_avg_position_mm()
        start_angle = self.get_avg_angle_deg()
        start_angle = self.get_avg_angle_deg()
        
        previous_time = time.ticks_ms()
        
        previous_position_delta = 0
        previous_angle_delta = 0
        
        #start_imu_yaw = self.imu.get_yaw()

        while True:
            current_position = self.get_avg_position_mm()
            current_position_delta = current_position - start_position
            target_velocity, velocity_state = speed_generator.get_target_velocity(current_position_delta)
            
            current_angle = self.get_avg_angle_deg()
            current_angle_delta = current_angle - start_angle
            target_omega, omega_state = omega_generator.get_target_velocity(current_angle_delta)

            current_time = time.ticks_ms()
            delta_t = time.ticks_diff(current_time, previous_time) / 1000
            
            error_position =  previous_position_delta + (target_velocity * delta_t) - current_position_delta
            error_angle = previous_angle_delta + (target_omega * delta_t) - current_angle_delta
            
            previous_time = current_time
            previous_position_delta = current_position_delta
            previous_angle_delta = current_angle_delta
            
            forward_effort = self.forward_pid.update(error_position) * self.flip_direction
            angle_effort = self.angle_pid.update(error_angle) * self.flip_angle
            #print(f"{velocity_state:2} {forward_effort:10.2f} {current_position_delta:10.2f} {error_position:10.2f}  {target_velocity:10.2f}")
            #print(f"{omega_state:2} {angle_effort:8.2f} {current_angle_delta:8.2f} {error_angle:8.2f}  {target_omega:8.2f} {(self.imu.get_yaw() - self.start_imu_yaw):10.2f} ")
            #print(f"{start_angle:2} {current_angle:8.2f} {current_angle_delta:8.2f} {previous_angle_delta:8.2f}  {target_omega:8.2f} {(error_angle):10.2f} ")
            #print(f"{target_distance - current_position_delta:2} {target_angle - current_angle_delta:2}")
            self.left_motor.set_effort(forward_effort + angle_effort)
            self.right_motor.set_effort(forward_effort - angle_effort)
            
            if (abs(target_distance - current_position_delta) <= 3.0 or target_distance == 0) and (abs(target_angle - current_angle_delta) <= 1.5 or target_angle == 0) : 
                break
            
            time.sleep(.01)

        def move_left(self,center_dist, left_dist, right_dist):
            if(center_dist < 20):
                dist_offset = (center_dist - 13) * 0.1 #cm to mm
            
            if(right_dist < 150):
                angle_offset = (right_dist - 70) * (5/20) #20 units = 5 deg correction?
            
            self.execute_profiles([
                [[182/2 + dist_offset, 800, 400, 50, 3200, 1],[0 + angle_offset, 1600, 0, 0, 2*3200, 1]],
                [[0, 800, 0, 0, 3200, 1],[90, 1600, 400, 200, 2*6400, -1]],
                [[182/2, 800, 400, 50, 3200, 1],[0, 1600, 0, 0, 2*3200, 1]],
                ])

        def move_forward(self,center_dist, left_dist, right_dist):
            if(left_dist < 150):
                angle_offset_left = (left_dist - 70) * (5/20) #20 units = 5 deg correction?
            
            if(right_dist < 150):
                angle_offset_right = (left_dist - 70) * (5/20) #20 units = 5 deg correction?
        
            self.execute_profiles([
                [[182/2, 800, 400, 50, 3200, 1],[0 + angle_offset_right - angle_offset_left, 1600, 0, 0, 2*3200, 1]],
                ])     
        
        def move_right(self,center_dist, left_dist, right_dist):
            if(center_dist < 20):
                dist_offset = (center_dist - 13) * 0.1 #cm to mm
            
            if(left_dist < 150):
                angle_offset = (left_dist - 70) * (5/20) #20 units = 5 deg correction?

            self.execute_profiles([
                [[182/2 + dist_offset, 800, 400, 50, 3200, 1],[0 + angle_offset, 1600, 0, 0, 2*3200, 1]],
                [[0, 800, 0, 0, 3200, 1],[90, 1600, 400, 200, 2*6400, 1]],
                [[182/2, 800, 400, 50, 3200, 1],[0, 1600, 0, 0, 2*3200, 1]],
                ])
    
        def move_turn_around(self,center_dist, left_dist, right_dist):
            self.execute_profiles([
                [[0, 800, 0, 0, 3200, 1],[180, 1600, 400, 200, 2*6400, 1]],
                ])    


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
