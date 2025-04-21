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

    def safety_off(self):
        """
        Clear the safety-switch latch so outputs are allowed
        (equivalent to pressing the safety button once).
        """
        self.vehicle.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SAFETY,
            0,
            0,      # param1: 0 = safety OFF, 1 = safety ON
            0,0,0,0,0,0)
        # Wait for the ACK so we know the autopilot accepted it
        ack = self.vehicle.recv_match(type='COMMAND_ACK',
                                    blocking=True, timeout=2)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Safety latch released")
        else:
            print("Safety command rejected!")
            
    def disable_arming_checks(self):
        """Disable arming checks through parameters"""
        print("Disabling arming checks...")
        
        # Set ARMING_CHECK parameter to 0 (disable all checks)
        self.vehicle.mav.param_set_send(
            self.target_system,
            self.target_component,
            b'ARMING_CHECK',  # Parameter name
            0,  # Parameter value (0 = disable all checks)
            mavutil.mavlink.MAV_PARAM_TYPE_INT32  # Parameter type
        )
        
        # Wait for parameter ACK
        ack = self.vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if ack and ack.param_id == b'ARMING_CHECK':
            print(f"ARMING_CHECK set to {ack.param_value}")
            return True
        else:
            print("Failed to set ARMING_CHECK parameter")
            return False

    
    def arm_vehicle(self, force=True):
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
        
    def set_GUIDED_NOGPS(self):
        # Optionally set GUIDED_NOGPS "GUID_OPTIONS" so it will accept attitude control
        print("Enabling external attitude control (GUID_OPTIONS = 8)...")
        self.vehicle.mav.param_set_send(
            self.target_system, self.target_component,
            b'GUID_OPTIONS',
            float(7),
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )

        # Switch mode to GUIDED_NOGPS
        print("Switching to GUIDED_NOGPS mode...")
        mode_id = self.vehicle.mode_mapping()['GUIDED_NOGPS']
        self.vehicle.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        time.sleep(1)
    
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
    
    def set_motor_output(self, motor_idx, throttle_percent, duration=5):
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
            duration,  # param4: timeout (10 seconds)
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
        
    def send_rt_command(self, cmd_motor_speeds, time, state, flat):
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
            self.set_motor_output(idx + 1, thrust_0100[idx])

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

    def set_all_motors_same(self, throttle_percent, duration):
        """
        Set all motors to the same throttle percentage for a given duration
        
        Args:
            throttle_percent: Motor output value (0-100 percent)
        """
        for motor in range(1, 5):
            self.set_motor_output(motor, throttle_percent, duration=duration)
    
    def stop_all_motors(self, duration=1):
        """Stop all motors"""
        self.set_all_motors_same(20, duration)
        # self.arm_vehicle()
        time.sleep(5)
        self.vehicle.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1, 0, 0, 0, 0, 0, 0
        )

    def close(self):
        """Close the MAVLink connection"""
        # Make sure to disarm before closing
        if self.is_armed:
            self.disarm_vehicle()
        
        self.vehicle.close()
        print("Connection closed")


    def relinquish_to_pilot(self):
        """
        • End DO_MOTOR_TEST
        • Put FCU in a stick‑driven mode
        • Return – pilot must arm with Tx
        """
        # a) send a single "all motors 0 %" test with zero timeout
        self.vehicle.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0, 0, 0,   # all motors, throttle‑type %
            0, 0,      # throttle 0, timeout 0 s  -> test ends immediately
            0,0,0)

        time.sleep(0.15)                 # let AP exit test state

        # b) pick a mode the pilot can fly while DISARMED
        self.vehicle.set_mode('STABILIZE')      # or 'ALT_HOLD' if you prefer
        print("Motor‑test ended, FCU in STABILIZE – arm with Tx to fly.")



    def _update_flight_mode(self):
        """
        Non-blocking read of the latest HEARTBEAT and extraction of the
        human-readable mode string (e.g. 'STABILIZE', 'ALT_HOLD', …).
        """
        hb = self.vehicle.recv_match(type='HEARTBEAT', blocking=False)
        if hb:
            # convenience property that pymavlink keeps up‑to‑date
            self.vehicle.flightmode = mavutil.mode_string_v10(hb)

    @property
    def flightmode(self):
        """Return the last known flight-mode string, or None."""
        self._update_flight_mode()
        return getattr(self.vehicle, "flightmode", None)
