#!/usr/bin/env python3
"""
Enhanced Motor Control Interface with Arming Functionality

This script provides functions to control motors using MAV_CMD_DO_MOTOR_TEST
and includes vehicle arming functionality.
"""
import time
import sys
from pymavlink import mavutil

class MotorController:
    def __init__(self, connection_string='/dev/ttyACM0'):
        """
        Initialize the MAVLink connection.
        
        Args:
            connection_string: MAVLink connection string
                - For USB: "/dev/ttyACM0" (Linux) or "COM3" (Windows)
                - For UDP: "udp:IP:PORT" (e.g., "udp:127.0.0.1:14550")
        """
        # Connect to the vehicle
        print(f"Connecting to vehicle on: {connection_string}")
        self.vehicle = mavutil.mavlink_connection(connection_string)
            
        # Wait for the first heartbeat
        self.vehicle.wait_heartbeat()
        print("Connected to vehicle.")
        
        # Store the target system and component IDs
        self.target_system = self.vehicle.target_system
        self.target_component = self.vehicle.target_component
        
        # Track arming state
        self.is_armed = False
    
    def get_available_modes_alternative(self):
        """
        Alternative method to get available flight modes
        """
        print("Requesting mode information...")
        
        # Request HEARTBEAT messages which contain mode information
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
            1000000,  # interval in microseconds (1 second)
            0, 0, 0, 0, 0  # unused parameters
        )
        
        # Wait for heartbeat messages
        print("Current mode information:")
        start_time = time.time()
        while time.time() - start_time < 5:  # 5 second timeout
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                mode = msg.custom_mode
                print(f"Current mode number: {mode}")
                return True
        
        print("No mode information received.")
        return False

    
    def arm_vehicle(self, force=False):
        """
        Arm the vehicle to allow motor operation
        
        Args:
            force: Whether to force arming regardless of pre-arm checks
            
        Returns:
            bool: True if arming was successful, False otherwise
        """
        print("Arming vehicle...")
        
        # Send arming command
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # 1 to arm, 0 to disarm
            21196 if force else 0,  # Force (21196 = 0x52C4 force arming)
            0, 0, 0, 0, 0  # unused parameters
        )
        
        # Wait for command ACK
        ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Vehicle armed successfully")
                self.is_armed = True
                return True
            else:
                print(f"Failed to arm vehicle. Result: {ack.result}")
                if ack.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                    print("Arming was temporarily rejected. Try using force=True or check pre-arm requirements.")
                return False
        else:
            print("No ACK received for arming command")
            return False
    
    def disarm_vehicle(self):
        """
        Disarm the vehicle
        
        Returns:
            bool: True if disarming was successful, False otherwise
        """
        print("Disarming vehicle...")
        
        # Send disarming command
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # 0 to disarm
            0, 0, 0, 0, 0, 0  # unused parameters
        )
        
        # Wait for command ACK
        ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Vehicle disarmed successfully")
            self.is_armed = False
            return True
        else:
            print("Failed to disarm vehicle")
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
        
        print(f"Setting motor {motor_idx} to {throttle_percent}%")
        
        # Check if vehicle is armed
        if throttle_percent > 0 and not self.is_armed:
            print("WARNING: Vehicle is not armed. Motors may not spin.")
            print("Call arm_vehicle() first to arm the vehicle.")
        
        # Send the command
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,  # confirmation
            motor_idx,  # param1: motor instance (1-based)
            mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,  # param2: throttle type
            throttle_percent,  # param3: throttle value (0-100%)
            10,  # param4: timeout (10 seconds)
            0, 0, 0  # unused parameters
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
            # time.sleep(0.1)  # Small delay between commands
    
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
        # Make sure to disarm before closing
        if self.is_armed:
            self.disarm_vehicle()
        
        self.vehicle.close()
        print("Connection closed")


# Example usage
if __name__ == "__main__":
    # Parse command line arguments
    if len(sys.argv) >= 2:
        controller = MotorController()
        
        if sys.argv[1] == "modes":
            # Check available modes
            controller.get_available_modes()
            controller.close()
            
        elif sys.argv[1] == "arm":
            # Arm the vehicle
            force = len(sys.argv) > 2 and sys.argv[2] == "force"
            controller.arm_vehicle(force=force)
            controller.close()
            
        elif sys.argv[1] == "disarm":
            # Disarm the vehicle
            controller.disarm_vehicle()
            controller.close()
            
        elif len(sys.argv) >= 3 and sys.argv[1].isdigit():
            # One-time motor control: python script.py motor_idx throttle_percent
            motor_idx = int(sys.argv[1])
            throttle_percent = float(sys.argv[2])
            
            # Arm the vehicle first
            if len(sys.argv) > 3 and sys.argv[3] == "noarm":
                print("Skipping arming as requested")
            else:
                controller.arm_vehicle()
            
            # Run the motor
            controller.set_motor_output(motor_idx, throttle_percent)
            time.sleep(1)
            print("going to 30")
            controller.set_motor_output(motor_idx, throttle_percent+10)
            time.sleep(1)
            print("going to 40")
            controller.set_motor_output(motor_idx, throttle_percent+20)
            time.sleep(1)
            print("going to 50")
            controller.set_motor_output(motor_idx, throttle_percent+30)
            time.sleep(1)
            print("going to 60")
            controller.set_motor_output(motor_idx, throttle_percent+40)
            time.sleep(1)
            print("going to 70")
            controller.set_motor_output(motor_idx, throttle_percent+50)
            time.sleep(5)  # Run for 2 seconds
            print("waited for 5 seconds")
            controller.set_motor_output(motor_idx, 0)  # Stop motor
            # Disarm and close
            controller.disarm_vehicle()
            controller.close()
            
        elif sys.argv[1] == "test":
            # Run test sequence
            print("Starting motor test sequence...")
            
            # Arm the vehicle first
            if len(sys.argv) > 2 and sys.argv[2] == "noarm":
                print("Skipping arming as requested")
            else:
                controller.arm_vehicle()
            
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
            time.sleep(3)
            controller.set_all_motors(40)
            time.sleep(3)
            controller.set_all_motors(50)
            time.sleep(3)  # Run for 3 seconds
            controller.stop_all_motors()
            
            print("Test complete!")
            
            # Disarm and close
            controller.disarm_vehicle()
            controller.close()
            
        else:
            print("Unknown command")
            controller.close()
    else:
        print("Usage:")
        print("  Check available modes: python script.py modes")
        print("  Arm vehicle: python script.py arm [force]")
        print("  Disarm vehicle: python script.py disarm")
        print("  One-time motor control: python script.py motor_idx throttle_percent [noarm]")
        print("  Example: python script.py 1 20  # Run motor 1 at 20% throttle")
        print("  Run test sequence: python script.py test [noarm]")
