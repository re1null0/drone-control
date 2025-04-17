#!/usr/bin/env python3

import time
import argparse
from pymavlink import mavutil

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------
connection_string = '/dev/ttyACM0'
baud_rate = 115200

# ------------------------------------------------------------
# Argument Parser
# ------------------------------------------------------------
parser = argparse.ArgumentParser(description='Arm or disarm the drone.')
parser.add_argument('--arm_status', type=int, required=True, choices=[0, 1],
                    help='Set to 0 to arm, 1 to disarm')
args = parser.parse_args()

# ------------------------------------------------------------
# Connect
# ------------------------------------------------------------
print(f"Connecting to {connection_string}...")
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master.wait_heartbeat()
print(f"Heartbeat received (system {master.target_system}, component {master.target_component})")

# ------------------------------------------------------------
# Functions
# ------------------------------------------------------------
def disable_arm_checks():
    print("Disabling arming checks...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'ARMING_CHECK',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(2)

def set_guided_options():
    print("Setting GUID_OPTIONS = 7...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'GUID_OPTIONS',
        float(7),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(2)

def set_mode(mode_name: str):
    print(f"Setting flight mode to {mode_name}...")
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    time.sleep(2)

def arm():
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors are armed.")

def disarm():
    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Motors are disarmed.")

# ------------------------------------------------------------
# Main Logic
# ------------------------------------------------------------
disable_arm_checks()
set_guided_options()
set_mode("GUIDED_NOGPS")

if args.arm_status == 0:
    arm()
else:
    disarm()
