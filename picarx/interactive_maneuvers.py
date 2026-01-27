#!/usr/bin/env python3
"""
Interactive Maneuvers Script for PiCar-X (Section 2.8 Task 2)
Allows keyboard control to execute discrete maneuvers.
"""

import sys
import tty
import termios
from maneuvers import Maneuvers


def get_key():
    """Read a single keypress from stdin without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def print_menu():
    """Print the control menu."""
    print("\n" + "=" * 50)
    print("        PiCar-X Interactive Maneuvers")
    print("=" * 50)
    print()
    print("  Basic Movement:")
    print("    [W] Forward")
    print("    [S] Backward")
    print("    [A] Forward with left turn")
    print("    [D] Forward with right turn")
    print()
    print("  Parallel Parking:")
    print("    [Q] Parallel park LEFT")
    print("    [E] Parallel park RIGHT")
    print()
    print("  Three-Point Turn (K-Turn):")
    print("    [Z] K-turn LEFT")
    print("    [C] K-turn RIGHT")
    print()
    print("  Control:")
    print("    [SPACE] Emergency STOP")
    print("    [X] Exit program")
    print()
    print("=" * 50)
    print("Waiting for input...", end=" ", flush=True)


def main():
    print("\nInitializing PiCar-X...")
    m = Maneuvers()
    
    print_menu()
    
    while True:
        key = get_key().lower()
        
        # Clear the "Waiting for input..." text
        print("\r" + " " * 30 + "\r", end="")
        
        if key == 'w':
            print(">> Forward")
            m.forward(speed=30, duration=1.0)
            
        elif key == 's':
            print(">> Backward")
            m.backward(speed=30, duration=1.0)
            
        elif key == 'a':
            print(">> Forward with LEFT turn")
            m.forward(speed=30, duration=1.0, angle=-25)
            
        elif key == 'd':
            print(">> Forward with RIGHT turn")
            m.forward(speed=30, duration=1.0, angle=25)
            
        elif key == 'q':
            print(">> Parallel park LEFT")
            m.parallel_park_left()
            
        elif key == 'e':
            print(">> Parallel park RIGHT")
            m.parallel_park_right()
            
        elif key == 'z':
            print(">> K-turn LEFT")
            m.k_turn_left()
            
        elif key == 'c':
            print(">> K-turn RIGHT")
            m.k_turn_right()
            
        elif key == ' ':
            print(">> STOP!")
            m.stop()
            
        elif key == 'x':
            print(">> Exiting...")
            m.stop()
            break
            
        elif key == '\x03':  # Ctrl+C
            print("\n>> Ctrl+C detected, stopping...")
            m.stop()
            break
        
        else:
            print(f"Unknown key: '{key}'")
        
        print("Waiting for input...", end=" ", flush=True)
    
    print("\nGoodbye!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted! Stopping motors...")
        m = Maneuvers()
        m.stop()
