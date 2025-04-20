#!/usr/bin/env python3
"""
Enhanced Motor Control Interface with Arming Functionality

This script provides functions to control motors using MAV_CMD_DO_MOTOR_TEST
and includes vehicle arming functionality.
"""
import time
import sys
from pymavlink import mavutil
import numpy as np

class MotorController:
    def __init__(self, quad_params, connection_string='/dev/ttyACM0'):
        """
        Initialize the MAVLink connection.
        
        Args:
            connection_string: MAVLink connection string
                - For USB: "/dev/ttyACM0" (Linux) or "COM3" (Windows)
                - For UDP: "udp:IP:PORT" (e.g., "udp:127.0.0.1:14550")
        """
        self.rotor_speed_max = quad_params["rotor_speed_max"]

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
        
    def send_rt_command(self, master, cmd_motor_speeds, time, state, flat):
        """
        Convert a desired quaternion & thrust [N] to
        roll/pitch/yaw in radians and a normalized thrust 0..1,
        then send to Pixhawk.
        cmd_q: [i, j, k, w] quaternion
        cmd_thrust: scalar in Newtons
        """
        thrust_0100 = np.clip(100 * cmd_motor_speeds / self.rotor_speed_max, 0, 100)

        # Ensure time is a scalar float
        t_val = float(np.squeeze(time))

        for idx in range(len(cmd_motor_speeds)):
            master.set_motor_output(idx + 1, thrust_0100[idx])

        np.set_printoptions(precision=3, suppress=True)
        info_line = (
            f"t={t_val:5.2f}   "
            f"pos=({state['x']})   "
            f"orientation({state['q']})" # i, j, k, w
            f"flat-({flat['x']})   "
            # f"thrust={cmd_thrust:5.2f}N => {thrust_01:4.2f}   "
            # f"oriento=({cmd_q})" # i, j, k, w
        )
        print(info_line)

    def set_all_motors_same(self, throttle_percent):
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

    def close(self):
        """Close the MAVLink connection"""
        # Make sure to disarm before closing
        if self.is_armed:
            self.disarm_vehicle()
        
        self.vehicle.close()
        print("Connection closed")