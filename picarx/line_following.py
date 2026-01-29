#!/usr/bin/env python3
"""
Line Following Module (Section 3)

Contains:
- Sensor: Reads grayscale sensor data (3.1)
- Interpreter: Converts sensor data to position [-1, 1] (3.2)
- Controller: Converts position to steering angle (3.3)
"""

import os
import logging

# Determine if we're running on the robot or in simulation
try:
    from robot_hat import ADC
    on_the_robot = True
except ImportError:
    import sys
    sys.path.append(os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..')))
    from sim_robot_hat import ADC
    on_the_robot = False


# =====================================================================
# SECTION 3.1: SENSOR CLASS
# =====================================================================

class Sensor:
    """
    Grayscale sensor reader for line following.
    
    Reads 3 ADC channels from the photosensors on the bottom of the robot.
    Higher values = lighter surface, Lower values = darker surface.
    """
    
    def __init__(self, pins=['A0', 'A1', 'A2']):
        """
        Initialize the sensor with ADC pins.
        
        Args:
            pins: List of 3 ADC pin names [left, center, right]
        """
        self.adc_left = ADC(pins[0])
        self.adc_center = ADC(pins[1])
        self.adc_right = ADC(pins[2])
        
        logging.debug(f"Sensor initialized on pins: {pins}")
    
    def read(self):
        """
        Read all three grayscale sensors.
        
        Returns:
            List of 3 values [left, center, right] (typically 0-4095)
        """
        left = self.adc_left.read()
        center = self.adc_center.read()
        right = self.adc_right.read()
        
        return [left, center, right]


# =====================================================================
# SECTION 3.2: INTERPRETER CLASS
# =====================================================================

class Interpreter:
    """
    Interprets grayscale sensor data into a position value.
    
    Converts raw sensor readings into a normalized position [-1, 1]
    where:
      -1 = line is far to the right (robot should turn right)
       0 = line is centered
      +1 = line is far to the left (robot should turn left)
    """
    
    def __init__(self, sensitivity=0.5, polarity='dark'):
        """
        Initialize the interpreter.
        
        Args:
            sensitivity: How different dark/light readings should be (0-1).
                        Higher = requires more contrast to detect edges.
            polarity: 'dark' if following a dark line on light background,
                     'light' if following a light line on dark background.
        """
        self.sensitivity = sensitivity
        self.polarity = polarity.lower()
        
        # Calibration values (can be set later)
        self.dark_ref = 0      # Expected reading on dark surface
        self.light_ref = 4095  # Expected reading on light surface
        
        logging.debug(f"Interpreter initialized: sensitivity={sensitivity}, polarity={polarity}")
    
    def set_references(self, dark_ref, light_ref):
        """
        Set calibration reference values.
        
        Args:
            dark_ref: Expected sensor reading on dark surface
            light_ref: Expected sensor reading on light surface
        """
        self.dark_ref = dark_ref
        self.light_ref = light_ref
        logging.debug(f"References set: dark={dark_ref}, light={light_ref}")
    
    def process(self, sensor_data):
        """
        Convert sensor readings to position value.
        
        Args:
            sensor_data: List of 3 sensor values [left, center, right]
        
        Returns:
            Position value in range [-1, 1]
            Positive = line is to the left (turn left)
            Negative = line is to the right (turn right)
        """
        left, center, right = sensor_data
        
        # Normalize readings to [0, 1] range
        # Lower value = darker (on the line for dark polarity)
        range_val = max(self.light_ref - self.dark_ref, 1)  # Avoid division by zero
        
        left_norm = (left - self.dark_ref) / range_val
        center_norm = (center - self.dark_ref) / range_val
        right_norm = (right - self.dark_ref) / range_val
        
        # Clamp to [0, 1]
        left_norm = max(0, min(1, left_norm))
        center_norm = max(0, min(1, center_norm))
        right_norm = max(0, min(1, right_norm))
        
        # For dark line on light background:
        # Low value = on line, High value = off line
        # If polarity is 'light', invert
        if self.polarity == 'light':
            left_norm = 1 - left_norm
            center_norm = 1 - center_norm
            right_norm = 1 - right_norm
        
        # Now: High = on line, Low = off line
        # Invert so high = on line
        left_on = 1 - left_norm
        center_on = 1 - center_norm
        right_on = 1 - right_norm
        
        # Calculate weighted position
        # If left sensor sees line more: position is positive (turn left)
        # If right sensor sees line more: position is negative (turn right)
        
        total = left_on + center_on + right_on
        
        if total < 0.01:
            # No line detected - return 0 or last known position
            return 0.0
        
        # Weighted average: left = +1, center = 0, right = -1
        position = (left_on * 1.0 + center_on * 0.0 + right_on * -1.0) / total
        
        # Clamp output to [-1, 1]
        position = max(-1.0, min(1.0, position))
        
        return position


# =====================================================================
# SECTION 3.3: CONTROLLER CLASS
# =====================================================================

class Controller:
    """
    Converts position error into steering commands.
    
    Uses proportional control to convert the position [-1, 1] 
    into a steering angle for the robot.
    """
    
    def __init__(self, scaling_factor=30.0):
        """
        Initialize the controller.
        
        Args:
            scaling_factor: Multiplier for position-to-angle conversion.
                           Default 30 means position=1 gives 30° steering.
        """
        self.scaling_factor = scaling_factor
        
        logging.debug(f"Controller initialized: scaling_factor={scaling_factor}")
    
    def control(self, position, picarx=None):
        """
        Calculate and optionally apply steering angle.
        
        Args:
            position: Position value from interpreter [-1, 1]
            picarx: Optional Picarx instance to apply steering
        
        Returns:
            Commanded steering angle (degrees)
        """
        # Position > 0 means line is to the left, so turn left (negative angle)
        # Position < 0 means line is to the right, so turn right (positive angle)
        steering_angle = -1 * position * self.scaling_factor
        
        # Clamp to servo limits
        steering_angle = max(-30, min(30, steering_angle))
        
        # Apply to robot if provided
        if picarx is not None:
            picarx.set_dir_servo_angle(steering_angle)
        
        return steering_angle


# =====================================================================
# SECTION 3.4: COMBINED LINE FOLLOWER
# =====================================================================

class LineFollower:
    """
    Combines Sensor, Interpreter, and Controller for complete line following.
    """
    
    def __init__(self, picarx=None, sensor_pins=['A0', 'A1', 'A2'],
                 sensitivity=0.5, polarity='dark', scaling_factor=30.0):
        """
        Initialize the complete line following system.
        
        Args:
            picarx: Picarx instance for motor control
            sensor_pins: ADC pins for grayscale sensors
            sensitivity: Interpreter sensitivity
            polarity: 'dark' or 'light' line
            scaling_factor: Controller gain
        """
        self.px = picarx
        self.sensor = Sensor(sensor_pins)
        self.interpreter = Interpreter(sensitivity, polarity)
        self.controller = Controller(scaling_factor)
    
    def calibrate(self, dark_ref, light_ref):
        """Set calibration values for the interpreter."""
        self.interpreter.set_references(dark_ref, light_ref)
    
    def update(self):
        """
        Perform one cycle of sense-interpret-control.
        
        Returns:
            Tuple of (sensor_data, position, steering_angle)
        """
        # Sense
        sensor_data = self.sensor.read()
        
        # Interpret
        position = self.interpreter.process(sensor_data)
        
        # Control
        steering_angle = self.controller.control(position, self.px)
        
        return sensor_data, position, steering_angle
    
    def follow(self, speed=30, duration=None, loop_delay=0.02):
        """
        Run continuous line following.
        
        Args:
            speed: Motor speed (0-100)
            duration: How long to run (seconds), None = forever
            loop_delay: Delay between updates (seconds)
        """
        import time
        
        if self.px is None:
            raise ValueError("Picarx instance required for follow()")
        
        start_time = time.time()
        
        try:
            self.px.forward(speed)
            
            while True:
                sensor_data, position, steering = self.update()
                
                logging.debug(f"Sensors: {sensor_data}, Position: {position:.2f}, Steering: {steering:.1f}°")
                
                if duration and (time.time() - start_time) >= duration:
                    break
                
                time.sleep(loop_delay)
                
        finally:
            self.px.stop()


# =====================================================================
# MAIN (for testing)
# =====================================================================

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s: %(message)s")
    
    print("Line Following Module")
    print("Run test_line_sensor.py for interactive testing")
