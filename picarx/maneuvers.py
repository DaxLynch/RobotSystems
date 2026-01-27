#!/usr/bin/env python3
"""
Maneuvers module for PiCar-X (Section 2.8)
Provides discrete movement actions: forward/backward, parallel parking, K-turns
"""

import time
from picarx_improved import Picarx


class Maneuvers:
    """High-level maneuver functions for the PiCar-X robot."""
    
    def __init__(self):
        self.px = Picarx()
        self.default_speed = 30
        self.default_duration = 1.0  # seconds
    
    # ==================== Basic Movement ====================
    
    def forward(self, speed=None, duration=None, angle=0):
        """
        Move forward in a straight line or with steering angle.
        
        Args:
            speed: Motor speed (0-100), defaults to self.default_speed
            duration: Time in seconds to move
            angle: Steering angle (-30 to 30), 0 = straight
        """
        speed = speed or self.default_speed
        duration = duration or self.default_duration
        
        self.px.set_dir_servo_angle(angle)
        self.px.forward(speed)
        time.sleep(duration)
        self.px.stop()
        self.px.set_dir_servo_angle(0)
    
    def backward(self, speed=None, duration=None, angle=0):
        """
        Move backward in a straight line or with steering angle.
        
        Args:
            speed: Motor speed (0-100), defaults to self.default_speed
            duration: Time in seconds to move
            angle: Steering angle (-30 to 30), 0 = straight
        """
        speed = speed or self.default_speed
        duration = duration or self.default_duration
        
        self.px.set_dir_servo_angle(angle)
        self.px.backward(speed)
        time.sleep(duration)
        self.px.stop()
        self.px.set_dir_servo_angle(0)
    
    # ==================== Parallel Parking ====================
    
    def parallel_park_left(self, speed=None):
        """
        Execute a parallel park to the left.
        Assumes starting position is parallel to the parking space.
        
        Sequence:
        1. Back up turning left (into the space)
        2. Back up turning right (straighten out)
        3. Pull forward to center
        """
        speed = 50 #This is a good speed for parallel parking, i.e. to get it to move one car distance over
        
        # Step 1: Back up while turning left
        self.px.set_dir_servo_angle(-30)
        self.px.backward(speed)
        time.sleep(1.2)
        self.px.stop()
        
        # Step 2: Back up while turning right to straighten
        self.px.set_dir_servo_angle(30)
        self.px.backward(speed)
        time.sleep(1.2)
        self.px.stop()
        
        # Step 3: Pull forward to center in spot
        self.px.set_dir_servo_angle(0)
        self.px.forward(speed)
        time.sleep(1.5)
        self.px.stop()
    
    def parallel_park_right(self, speed=None):
        """
        Execute a parallel park to the right.
        Assumes starting position is parallel to the parking space.
        
        Sequence:
        1. Back up turning right (into the space)
        2. Back up turning left (straighten out)
        3. Pull forward to center
        """
        speed = 50 #This is a good speed for parallel parking, i.e. to get it to move one car distance over
        
        #Step 0: Stop the car
        self.px.stop()
        
        # Step 1: Back up while turning right
        self.px.set_dir_servo_angle(30)
        self.px.backward(speed)
        time.sleep(1.2)
        self.px.stop()
        
        # Step 2: Back up while turning left to straighten
        self.px.set_dir_servo_angle(-30)
        self.px.backward(speed)
        time.sleep(1.2)
        self.px.stop()
        
        # Step 3: Pull forward to center in spot
        self.px.set_dir_servo_angle(0)
        self.px.forward(speed)
        time.sleep(1.5)
        self.px.stop()
    
    # ==================== Three-Point Turn (K-Turn) ====================
    
    def k_turn_left(self):
        """
        Execute a three-point turn (K-turn) starting with a left turn.
        Used to turn around 180 degrees in a confined space.
        
        Sequence:
        1. Turn left while moving forward
        2. Turn right while backing up
        3. Straighten and move forward
        """
        speed = 50
        
        # Step 1: Forward while turning hard left
        self.px.set_dir_servo_angle(-30)
        self.px.forward(speed)
        time.sleep(1.5)
        self.px.stop()
        
        # Step 2: Reverse while turning hard right
        self.px.set_dir_servo_angle(30)
        self.px.backward(speed)
        time.sleep(1.5)
        self.px.stop()
        
        # Step 3: Forward to complete the turn
        self.px.set_dir_servo_angle(-30)
        self.px.forward(speed)
        time.sleep(1.2)
        self.px.stop()
        
        # Straighten wheels
        self.px.set_dir_servo_angle(0)
    
    def k_turn_right(self):
        """
        Execute a three-point turn (K-turn) starting with a right turn.
        Used to turn around 180 degrees in a confined space.
        
        Sequence:
        1. Turn right while moving forward
        2. Turn left while backing up
        3. Straighten and move forward
        """
        speed = 50
        
        # Step 1: Forward while turning hard right
        self.px.set_dir_servo_angle(30)
        self.px.forward(speed)
        time.sleep(1.5)
        self.px.stop()
        
        # Step 2: Reverse while turning hard left
        self.px.set_dir_servo_angle(-30)
        self.px.backward(speed)
        time.sleep(1.5)
        self.px.stop()
        
        # Step 3: Forward to complete the turn
        self.px.set_dir_servo_angle(30)
        self.px.forward(speed)
        time.sleep(1.2)
        self.px.stop()
        
        # Straighten wheels
        self.px.set_dir_servo_angle(0)
    
    def stop(self):
        """Stop all movement immediately."""
        self.px.stop()
        self.px.set_dir_servo_angle(0)


# Test the maneuvers if run directly
if __name__ == "__main__":
    print("Maneuvers Test")
    print("=" * 40)
    
    m = Maneuvers()
    m.k_turn_left()

