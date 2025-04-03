#!/usr/bin/env python3

"""
Example script: Offboard Attitude Control in GUIDED_NOGPS mode using pymavlink
--------------------------------------------------------------------------------
- Connects to an ArduCopter vehicle (UDP or serial).
- Disables arming checks, sets GUIDED_NOGPS mode, arms the motors.
- Sends attitude (roll/pitch/yaw) and thrust setpoints via SET_ATTITUDE_TARGET.
- Demonstrates small pitch changes and then a neutral hover attitude.
--------------------------------------------------------------------------------
NOTE: You must have at least a barometer or stable IMU for the EKF to allow flight.
      For SITL, the default environment typically has a baro + IMU simulated.
      In real hardware, you do this at your own risk if you truly have no GPS.
"""

import time
import math
from pymavlink import mavutil

# CHANGE THIS: If you're running SITL locally, use 'udp:127.0.0.1:14551'
# If you're on real hardware over USB, something like '/dev/ttyACM0' (Linux) or 'COM3' (Windows).
connection_string = 'udp:127.0.0.1:14551'
baud_rate = 115200

boot_time = time.time()

# Connect to the vehicle
print(f"Connecting to {connection_string}...")
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master.wait_heartbeat()
print("Heartbeat received from system "
      f"(system {master.target_system} component {master.target_component})")

# -------------------------------------------------------------------
# Helper functions
# -------------------------------------------------------------------

def disable_arm_checks():
    """Disable arming checks so we can arm without GPS, etc."""
    print("Disabling arming checks...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'ARMING_CHECK',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(2)

def set_guided_options():
    """
    Some ArduPilot versions require GUID_OPTIONS to enable attitude or yaw/thrust control 
    in guided modes. Setting it to 7 (binary 0b0111) often allows external attitude, yaw, 
    and throttle control.
    """
    print("Setting GUID_OPTIONS = 7 to allow external attitude control...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'GUID_OPTIONS',
        float(7),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(2)

def set_mode(mode_name: str):
    """Set the desired flight mode."""
    print(f"Setting flight mode to {mode_name}...")
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

def arm():
    """Arm the drone motors."""
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors are armed!")

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """
    Convert Euler angles (roll, pitch, yaw) in radians
    to a quaternion [w, x, y, z].
    """
    cr2 = math.cos(roll / 2.0)
    cp2 = math.cos(pitch / 2.0)
    cy2 = math.cos(yaw / 2.0)
    sr2 = math.sin(roll / 2.0)
    sp2 = math.sin(pitch / 2.0)
    sy2 = math.sin(yaw / 2.0)

    w = cr2 * cp2 * cy2 + sr2 * sp2 * sy2
    x = sr2 * cp2 * cy2 - cr2 * sp2 * sy2
    y = cr2 * sp2 * cy2 + sr2 * cp2 * sy2
    z = cr2 * cp2 * sy2 - sr2 * sp2 * cy2
    return [w, x, y, z]

def send_attitude_target(roll_deg: float, pitch_deg: float, yaw_deg: float, thrust: float):
    """
    Send an attitude setpoint (roll, pitch, yaw in degrees, thrust 0..1).
    ArduPilot interprets thrust ~0.5 as hover, depends on many factors.

    We ignore body rates and only use orientation+thrust control.
    """
    # Convert degrees to radians
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    # Build quaternion from Euler angles
    q = euler_to_quaternion(roll, pitch, yaw)

    # Bitmask:
    #  bit 0: body roll rate
    #  bit 1: body pitch rate
    #  bit 2: body yaw rate
    #  bit 6: attitude (orientation)
    #  bit 7: thrust
    # We want to use attitude+thrust, ignore body rates:
    type_mask = (1 << 0) | (1 << 1) | (1 << 2)  # = 7, ignoring roll/pitch/yaw rate

    # TIMESTAMP in milliseconds
    now_ms = int((time.time() - boot_time) * 1000)

    master.mav.set_attitude_target_send(
        now_ms,
        master.target_system,
        master.target_component,
        type_mask,
        q,   # Quaternion of desired orientation
        0.0, 0.0, 0.0,  # roll_rate, pitch_rate, yaw_rate (ignored)
        thrust
    )

# -------------------------------------------------------------------
# Main logic
# -------------------------------------------------------------------

# 1) Disable arming checks so we can arm without GPS
disable_arm_checks()

# 2) Optionally set GUIDED_NOGPS "GUID_OPTIONS" so it will accept attitude control
set_guided_options()

# 3) Switch to GUIDED_NOGPS
set_mode("GUIDED_NOGPS")
time.sleep(2)

# 4) Arm
arm()

# 5) Send attitude setpoints for a small demonstration
#    We'll do a loop sending repeated commands to maintain the attitude.

print("Taking off: we'll try to hover at ~0.6 thrust for 5 seconds.")
start_time = time.time()
while time.time() - start_time < 5:
    # Keep roll=0, pitch=0, yaw=0, thrust=0.6 (slightly above hover for many vehicles)
    send_attitude_target(0, 0, 0, 0.6)
    time.sleep(0.1)

print("Pitching forward by ~5 degrees for 3 seconds...")
start_time = time.time()
while time.time() - start_time < 6:
    # Pitch forward slightly, same thrust
    send_attitude_target(0.55, 0, 0, 0.5)
    time.sleep(0.1)

print("Returning to level hover for 3 seconds...")
start_time = time.time()
while time.time() - start_time < 3:
    send_attitude_target(0, 0, 0, 0.6)
    time.sleep(0.1)

print("Now reducing thrust to descend gently for 3 seconds.")
start_time = time.time()
while time.time() - start_time < 3:
    # Lower thrust to ~0.4, hopefully descending
    send_attitude_target(0, 0, 0, 0.4)
    time.sleep(0.1)

print("Disarming motors...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,  # 0 = disarm
    0, 0, 0, 0, 0, 0
)
master.motors_disarmed_wait()
print("Motors are disarmed. Script complete.")
