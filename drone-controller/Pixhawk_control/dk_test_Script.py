import time 
import os
import platform
import sys

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
import numpy as np

vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)
print("Connected to vehicle on: %s" % vehicle._connection_string)
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


def arm():
    while vehicle.is_armable is False:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
    print("Vehicle is armable, setting to GUIDED mode... \n")
    
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)
    print("Vehicle is in GUIDED mode, arming... \n")
    
    vehicle.armed = True
    while vehicle.armed is False:
        print("Waiting for vehicle to arm...")
        time.sleep(1)
    print("Vehicle is armed, ready to fly! \n")
    
arm()