#!/usr/bin/env python3
"""
Test script for line following sensor (Section 3.4)

Continuously reads grayscale sensors and outputs:
- Raw sensor values [left, center, right]
- Interpreted position [-1 to +1]

Position interpretation:
  +1 = line is to the LEFT (robot should turn left)
   0 = line is CENTERED
  -1 = line is to the RIGHT (robot should turn right)
"""

import time
import sys
from line_following import Sensor, Interpreter

# Default grayscale sensor pins
DEFAULT_PINS = ['A0', 'A1', 'A2']


def auto_calibrate(sensor, samples=3*50, delay=0.02):
    """
    Simple auto-calibration routine.
    
    Collects samples and uses min/max to estimate dark/light references.
    Move the sensors over both dark and light areas during this time.
    """
    print("\n=== AUTO-CALIBRATION ===")
    print("Move the robot over the LINE and the BACKGROUND for 3 seconds...")
    print("Collecting samples", end="", flush=True)
    
    all_readings = []
    for i in range(samples):
        all_readings.append(sensor.read())
        print(".", end="", flush=True)
        time.sleep(delay)
    
    print(" Done!")
    
    # Find min/max across all readings
    all_values = [v for reading in all_readings for v in reading]
    dark_ref = min(all_values)
    light_ref = max(all_values)
    
    print(f"Detected range: dark={dark_ref}, light={light_ref}")
    
    return dark_ref, light_ref


def manual_calibrate(sensor):
    """
    Manual calibration by prompting user.
    """
    print("\n=== MANUAL CALIBRATION ===")
    
    input("Place sensors over the DARK LINE, then press Enter...")
    dark_readings = []
    for _ in range(20):
        dark_readings.append(sensor.read())
        time.sleep(0.02)
    dark_avg = [sum(r[i] for r in dark_readings) // len(dark_readings) for i in range(3)]
    dark_ref = min(dark_avg)
    print(f"Dark readings (avg): {dark_avg}")
    
    input("Place sensors over the LIGHT BACKGROUND, then press Enter...")
    light_readings = []
    for _ in range(20):
        light_readings.append(sensor.read())
        time.sleep(0.02)
    light_avg = [sum(r[i] for r in light_readings) // len(light_readings) for i in range(3)]
    light_ref = max(light_avg)
    print(f"Light readings (avg): {light_avg}")
    
    return dark_ref, light_ref


def print_bar(position, width=40):
    """
    Print a visual bar showing position.
    
    [--------------------X--------------------]
                         ^ center
    """
    # Map position [-1, 1] to index [0, width]
    idx = int((position + 1) / 2 * width)
    idx = max(0, min(width, idx))
    
    bar = ['-'] * (width + 1)
    bar[width // 2] = '|'  # Center marker
    bar[idx] = 'X'
    
    return '[' + ''.join(bar) + ']'


def main():
    print("=" * 60)
    print("       LINE SENSOR TEST - Section 3.4")
    print("=" * 60)
    print()
    print("This script reads the grayscale sensors and outputs")
    print("the interpreted line position as a value from -1 to +1")
    print()
    print("  +1 = line is to the LEFT")
    print("   0 = line is CENTERED")
    print("  -1 = line is to the RIGHT")
    print()
    
    # Initialize sensor
    print(f"Initializing sensors on pins {DEFAULT_PINS}...")
    sensor = Sensor(DEFAULT_PINS)
    
    # Initialize interpreter (dark line on light background)
    # line_lost_threshold: if ALL sensors read above this, line is considered lost
    LINE_LOST_THRESHOLD = 1000
    interpreter = Interpreter(sensitivity=0.5, polarity='dark', line_lost_threshold=LINE_LOST_THRESHOLD)
    print(f"Line lost threshold: {LINE_LOST_THRESHOLD}")
    
    # Calibration
    print("\nCalibration options:")
    print("  [1] Auto-calibrate (move robot over line and background)")
    print("  [2] Manual calibrate (prompted)")
    print("  [3] Skip (use defaults)")
    print()
    
    try:
        choice = input("Select option (1/2/3): ").strip()
    except EOFError:
        choice = '3'
    
    if choice == '1':
        dark_ref, light_ref = auto_calibrate(sensor)
    elif choice == '2':
        dark_ref, light_ref = manual_calibrate(sensor)
    else:
        # Default values - adjust based on your sensors
        dark_ref = 200
        light_ref = 1500
        print(f"\nUsing default calibration: dark={dark_ref}, light={light_ref}")
    
    interpreter.set_references(dark_ref, light_ref)
    
    # Main loop
    print("\n" + "=" * 60)
    print("Starting sensor loop... (Ctrl+C to stop)")
    print("=" * 60)
    print()
    print("  Raw Sensors [L, C, R]      Position  Status  Visual")
    print("-" * 70)
    
    try:
        while True:
            # Read sensors
            sensor_data = sensor.read()
            
            # Interpret position - returns (position, line_visible)
            position, line_visible = interpreter.process(sensor_data)
            
            # Format output
            sensor_str = f"[{sensor_data[0]:4d}, {sensor_data[1]:4d}, {sensor_data[2]:4d}]"
            pos_str = f"{position:+.3f}"
            status = "LINE" if line_visible else "LOST"
            bar_str = print_bar(position)
            
            # Print on same line (overwrite)
            print(f"\r  {sensor_str}    {pos_str}  {status}   {bar_str}", end="", flush=True)
            
            time.sleep(0.05)  # 20 Hz update rate
            
    except KeyboardInterrupt:
        print("\n\nStopped by user.")


if __name__ == "__main__":
    main()
