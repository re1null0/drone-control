import time 
import os
import platform
import sys

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
import numpy as np


# Check if '/dev/ttyACM0' exists; if not, use '/dev/ttyACM1'
connection_string = '/dev/ttyACM0'
#if not os.path.exists(connection_string):
#    connection_string = '/dev/ttyACM1'

vehicle = connect(connection_string, baud=115200, wait_ready=True)
print("Connected to vehicle on: %s" % connection_string)

# -------------------------------------------
# Disable crash detection (NOT RECOMMENDED!)
# -------------------------------------------
'''try:
    print("Current FS_CRASH_CHECK =", vehicle.parameters['FS_CRASH_CHECK'])
    print("Disabling crash detection (FS_CRASH_CHECK = 0) ...")
    vehicle.parameters['FS_CRASH_CHECK'] = 0
    time.sleep(1)  # Give the autopilot time to set the parameter
    print("New FS_CRASH_CHECK =", vehicle.parameters['FS_CRASH_CHECK'])
except KeyError:
    print("FS_CRASH_CHECK parameter not found. This autopilot firmware may not support it.")
except Exception as e:
    print(f"Error setting FS_CRASH_CHECK: {e}")
'''
print(str(vehicle.system_status.state))
print("Vehicle mode: %s" % vehicle.mode.name)
print("Vehicle armed: %s" % vehicle.armed)
print("Vehicle location: %s" % vehicle.location.global_frame)
print("Vehicle velocity: %s" % vehicle.velocity)
print("Vehicle heading: %s" % vehicle.heading)
print("Vehicle battery: %s" % vehicle.battery)
print("Vehicle GPS: %s" % vehicle.gps_0)
print("Vehicle ground speed: %s" % vehicle.groundspeed)
print("Vehicle airspeed: %s" % vehicle.airspeed)
print("Vehicle attitude: %s" % vehicle.attitude)
print("Vehicle altitude: %s" % vehicle.location.global_relative_frame.alt)


def arm_takeoff(takeoff=True, target_height=1):
    # while vehicle.is_armable is False:
    #     print("Waiting for vehicle to initialize...")
    #     time.sleep(1)
    print("Vehicle is armable, setting to GUIDED mode... \n")
    
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)
    print("Vehicle is in GUIDED mode, arming... \n")
    
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)
    print(f"Vehicle is armed, ready to fly to {target_height}! \n")

    vehicle.simple_takeoff(target_height)
    while takeoff:
        current_altitude = vehicle.location.global_relative_frame.alt
        print("Current Altitude: " + str(current_altitude))

        if current_altitude >= 0.98 * target_height:
            break
        time.sleep(0.5)

    return None

arm_takeoff()

vehicle.mode = VehicleMode("LAND")
while vehicle.mode!='LAND':
    time.sleep(1)
    print("Waiting for drone to enter LAND mode")

print("Drone is in LAND mode. Exiting")
