#!/usr/bin/env python3
"""
CUSTOM_CTRL Motor Control Interface

This script provides functions to control motors in CUSTOM_CTRL mode using MAV_CMD_DO_MOTOR_TEST.
It includes both one-time control commands and continuous control capabilities.
"""
import time
import sys
from pymavlink import mavutil

class MotorController:
    def __init__(self, connection_string='COM8', baud=115200):
        """
        Initialize the MAVLink connection.
        
        Args:
            connection_string: MAVLink connection string
                - For UDP: "udp:IP:PORT" (e.g., "udp:127.0.0.1:14550")
                - For serial: "/dev/ttyACM0" or "COM3" on Windows
        """
        # Connect to the vehicle
        print(f"Connecting to vehicle on: {connection_string}")
        self.vehicle = mavutil.mavlink_connection(connection_string, baud)
            
        # Wait for the first heartbeat
        self.vehicle.wait_heartbeat()
        print("Connected to vehicle.")
        
        # Store the target system and component IDs
        self.target_system = self.vehicle.target_system
        self.target_component = self.vehicle.target_component
    
    def set_mode_custom_control(self):
        """
        Set the vehicle mode to CUSTOM_CTRL.
        
        Returns:
            bool: True if mode change was successful, False otherwise
        """
        # CUSTOM_CTRL mode number is 29
        mode_id = 29
        
        # Send mode change command
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0, 0, 0, 0, 0  # unused parameters
        )
        
        # Wait for command ACK
        ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Successfully changed to CUSTOM_CTRL mode")
            return True
        else:
            print("Failed to change to CUSTOM_CTRL mode")
            return False
    
    def set_motor_output(self, motor_idx, throttle_percent):
        """
        Set individual motor output using DO_MOTOR_TEST command
        
        Args:
            motor_idx: Motor index (1-4 for quadcopter, 1-based index)
            throttle_percent: Motor output value (0-100 percent)
            
        Returns:
            bool: True if command was accepted, False otherwise
        """
        # Ensure values are within bounds
        motor_idx = max(1, min(4, motor_idx))
        throttle_percent = max(0, min(100, throttle_percent))
        
        # Send the command
        self.vehicle.mav.command_int_send(
            self.target_system,
            self.target_component,
            0,  # frame (not used)
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,  # current (not used)
            0,  # autocontinue (not used)
            motor_idx,  # param1: motor instance (1-based)
            0,  # param2: throttle type (0=throttle percentage)
            throttle_percent,  # param3: throttle value (0-100%)
            0,  # param4: timeout (not used)
            0, 0, 0  # x, y, z (not used)
        )
        
        # Wait for command ACK
        ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Motor {motor_idx} set to {throttle_percent}% - Command accepted")
                return True
            else:
                print(f"Command failed with result: {ack.result}")
                return False
        else:
            print("No ACK received")
            return False
    
    def set_all_motors(self, throttle_percent):
        """
        Set all motors to the same throttle percentage
        
        Args:
            throttle_percent: Motor output value (0-100 percent)
        """
        for motor in range(1, 5):
            self.set_motor_output(motor, throttle_percent)
            time.sleep(0.1)  # Small delay between commands
    
    def stop_all_motors(self):
        """Stop all motors"""
        self.set_all_motors(0)
    
    def run_motor_sequence(self, sequence, delay=1.0):
        """
        Run a sequence of motor commands
        
        Args:
            sequence: List of (motor_idx, throttle_percent, duration) tuples
            delay: Delay between commands in seconds
        """
        try:
            for motor_idx, throttle_percent, duration in sequence:
                print(f"Setting motor {motor_idx} to {throttle_percent}% for {duration} seconds")
                self.set_motor_output(motor_idx, throttle_percent)
                time.sleep(duration)
                self.set_motor_output(motor_idx, 0)  # Stop motor
                time.sleep(delay)
        except KeyboardInterrupt:
            print("Interrupted! Stopping all motors")
            self.stop_all_motors()
    
    def close(self):
        """Close the MAVLink connection"""
        self.vehicle.close()
        print("Connection closed")


# Example usage for one-time motor control
if __name__ == "__main__":
    # Parse command line arguments
    if len(sys.argv) == 3:
        # One-time motor control: python script.py motor_idx throttle_percent
        motor_idx = int(sys.argv[1])
        throttle_percent = float(sys.argv[2])
        
        controller = MotorController()
        controller.set_mode_custom_control()
        controller.set_motor_output(motor_idx, throttle_percent)
        time.sleep(2)  # Run for 2 seconds
        controller.set_motor_output(motor_idx, 0)  # Stop motor
        controller.close()
        
    elif len(sys.argv) == 2 and sys.argv[1] == "test":
        # Run test sequence
        controller = MotorController()
        controller.set_mode_custom_control()
        
        print("Starting motor test sequence...")
        
        # Test each motor individually
        sequence = [
            (1, 20, 2),  # Motor 1, 20% throttle, 2 seconds
            (2, 20, 2),  # Motor 2, 20% throttle, 2 seconds
            (3, 20, 2),  # Motor 3, 20% throttle, 2 seconds
            (4, 20, 2),  # Motor 4, 20% throttle, 2 seconds
        ]
        controller.run_motor_sequence(sequence)
        
        # Test all motors together
        print("Testing all motors together")
        controller.set_all_motors(30)  # 30% throttle
        time.sleep(3)  # Run for 3 seconds
        controller.stop_all_motors()
        
        print("Test complete!")
        controller.close()
        
    else:
        print("Usage:")
        print("  One-time motor control: python script.py motor_idx throttle_percent")
        print("  Example: python script.py 1 20  # Run motor 1 at 20% throttle")
        print("  Run test sequence: python script.py test")
