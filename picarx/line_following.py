#!/usr/bin/env python3
"""
Line Following Module (Section 3)

Contains:
- Sensor: Reads grayscale sensor data (3.1)
- Interpreter: Converts sensor data to position [-1, 1] (3.2)
- Controller: Converts position to steering angle (3.3)
"""

import os
import time
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
# SECTION 3.3: PID CONTROLLER CLASS
# =====================================================================

class Controller:
    """
    PID Controller for line following.
    
    Converts position error [-1, 1] into steering angle.
    
    Speed scaling:
    - P term: Base gain (not scaled by speed)
    - D term: Scaled by speed (faster = more derivative response needed)
    - I term: Not scaled (accumulates error over time)
    """
    
    def __init__(self, kp=30.0, ki=0.0, kd=0.0):
        """
        Initialize the PID controller.
        
        Args:
            kp: Proportional gain. position=1 with kp=30 gives 30° steering.
            ki: Integral gain. Corrects accumulated error over time.
            kd: Derivative gain. Dampens rapid changes in error.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        
        logging.debug(f"PID Controller initialized: kp={kp}, ki={ki}, kd={kd}")
    
    def reset(self):
        """Reset PID state (integral and previous error)."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
    
    def control(self, position, speed=30, picarx=None):
        """
        Calculate PID control output.
        
        Args:
            position: Position error from interpreter [-1, 1]
                     Positive = line is left, Negative = line is right
            speed: Current forward speed (used to scale D term)
            picarx: Optional Picarx instance to apply steering
        
        Returns:
            Commanded steering angle (degrees)
        """
        import time
        
        # Error: we want position = 0 (centered)
        # If position > 0, line is left, we need to turn left (negative steering)
        error = position
        
        # Time delta for I and D terms
        current_time = time.time()
        if self.prev_time is None:
            dt = 0.02  # Default to 50Hz
        else:
            dt = current_time - self.prev_time
            dt = max(dt, 0.001)  # Prevent division by zero
        self.prev_time = current_time
        
        # P term (not scaled by speed)
        p_term = self.kp * error
        
        # I term (accumulate error, with anti-windup)
        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))  # Anti-windup clamp
        i_term = self.ki * self.integral
        
        # D term (scaled by speed - faster = more derivative response)
        # At higher speeds, the same physical deviation happens faster
        # so we need more aggressive derivative response
        derivative = (error - self.prev_error) / dt
        speed_scale = speed / 30.0  # Normalize: speed=30 gives scale=1.0
        d_term = self.kd * derivative * speed_scale
        
        self.prev_error = error
        
        # Total steering command
        # Negative because positive error (line left) needs left turn (negative angle)
        steering_angle = -1 * (p_term + i_term + d_term)
        
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
                 sensitivity=0.5, polarity='dark', kp=30.0, ki=0.0, kd=0.0):
        """
        Initialize the complete line following system.
        
        Args:
            picarx: Picarx instance for motor control
            sensor_pins: ADC pins for grayscale sensors
            sensitivity: Interpreter sensitivity
            polarity: 'dark' or 'light' line
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.px = picarx
        self.sensor = Sensor(sensor_pins)
        self.interpreter = Interpreter(sensitivity, polarity)
        self.controller = Controller(kp, ki, kd)
        self.speed = 30  # Default speed
    
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
        
        # Control (pass speed for D term scaling)
        steering_angle = self.controller.control(position, self.speed, self.px)
        
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
        
        self.speed = speed
        self.controller.reset()  # Reset PID state
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
# MAIN - 50Hz PID Line Following Loop
# =====================================================================

def main():
    """
    Run PID-based line following at 50Hz.
    
    Calibration: dark=200, light=1500
    """
    import time
    import atexit
    
    # Import picarx
    from picarx_improved import Picarx
    
    # ==================== CONFIGURATION ====================
    # Calibration values
    DARK_REF = 200
    LIGHT_REF = 1500
    
    # PID gains (start with P only, tune from there)
    KP = 25.0   # Proportional: how hard to steer based on error
    KI = 0.0    # Integral: correct accumulated drift (start at 0)
    KD = 0.0    # Derivative: dampen oscillations (start at 0)
    
    # Speed and loop settings
    FORWARD_SPEED = 30  # Motor speed (0-100)
    LOOP_HZ = 50        # Control loop frequency
    LOOP_DELAY = 1.0 / LOOP_HZ  # 0.02 seconds = 20ms
    
    # =======================================================
    
    print("=" * 60)
    print("       PID LINE FOLLOWING - 50Hz Loop")
    print("=" * 60)
    print()
    print(f"  Calibration: dark={DARK_REF}, light={LIGHT_REF}")
    print(f"  PID Gains:   Kp={KP}, Ki={KI}, Kd={KD}")
    print(f"  Speed:       {FORWARD_SPEED}")
    print(f"  Loop Rate:   {LOOP_HZ} Hz ({LOOP_DELAY*1000:.1f} ms)")
    print()
    
    # Initialize robot
    print("Initializing PiCar-X...")
    px = Picarx()
    
    # Ensure we stop on exit
    atexit.register(px.stop)
    
    # Initialize sensor and interpreter
    sensor = Sensor(['A0', 'A1', 'A2'])
    interpreter = Interpreter(polarity='dark')
    interpreter.set_references(DARK_REF, LIGHT_REF)
    
    # Initialize PID controller
    controller = Controller(kp=KP, ki=KI, kd=KD)
    
    print("Starting in 2 seconds... (Ctrl+C to stop)")
    time.sleep(2)
    
    print()
    print("  Position   Steering   [Left, Center, Right]")
    print("-" * 60)
    
    try:
        # Start moving forward
        px.forward(FORWARD_SPEED)
        
        loop_count = 0
        
        while True:
            loop_start = time.time()
            
            # 1. SENSE: Read grayscale sensors
            sensor_data = sensor.read()
            
            # 2. INTERPRET: Convert to position [-1, +1]
            position = interpreter.process(sensor_data)
            
            # 3. CONTROL: PID to steering angle
            steering = controller.control(position, FORWARD_SPEED, px)
            
            # Print status every 10 loops (~5Hz display update)
            if loop_count % 10 == 0:
                print(f"\r  {position:+.3f}      {steering:+6.1f}°     [{sensor_data[0]:4d}, {sensor_data[1]:4d}, {sensor_data[2]:4d}]", 
                      end="", flush=True)
            
            loop_count += 1
            
            # Maintain loop timing
            elapsed = time.time() - loop_start
            sleep_time = LOOP_DELAY - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        px.stop()
        px.set_dir_servo_angle(0)  # Center steering
        print("Done!")


if __name__ == "__main__":
    main()
