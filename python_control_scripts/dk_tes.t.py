from dronekit import connect, VehicleMode
import time

# Connect to the Pixhawk via serial
vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=115200)

def disable_safety_checks():
    print("🔧 Disabling safety checks...")
    vehicle.parameters['ARMING_CHECK'] = 0
    vehicle.parameters['FS_THR_ENABLE'] = 1
    vehicle.parameters['ARMING_REQUIRE'] = 0
    vehicle.parameters['GPS_TYPE'] = 0  # If using GUIDED_NOGPS
    time.sleep(2)

def set_guided_nogps_mode():
    print("🎛️ Setting mode to GUIDED_NOGPS...")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    while not vehicle.mode.name == "GUIDED_NOGPS":
        print("⏳ Waiting for mode change...")
        time.sleep(1)
    print("✅ Mode set to GUIDED_NOGPS")

def arm():
    print("🔋 Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print("⏳ Waiting for arming...")
        time.sleep(1)
    print("✅ Motors armed")

def takeoff(altitude):
    print(f"🚀 Taking off to {altitude} meters")
    vehicle.simple_takeoff(altitude)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"🛫 Current Altitude: {current_alt:.2f} m")
        if current_alt >= altitude * 0.95:
            print("✅ Target altitude reached")
            break
        time.sleep(1)

def land():
    print("🛬 Landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("⏳ Waiting for landing...")
        time.sleep(1)
    print("✅ Drone landed and disarmed")

# ---- MAIN ----
disable_safety_checks()
set_guided_nogps_mode()
arm()
takeoff(altitude=1.5)  # Target altitude (meters)

print("⏱️ Hovering...")
time.sleep(5)  # Hover duration

land()
vehicle.close()
