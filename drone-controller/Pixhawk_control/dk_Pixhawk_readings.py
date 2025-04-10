from dronekit import connect, VehicleMode
import time

# Adjust the connection string and baud rate for your setup.
connection_string = '/dev/ttyACM0'
vehicle = connect(connection_string, wait_ready=True, baud=57600)

try:
    while True:
        output = (
            f"\nState: {vehicle.system_status.state}\n"
            f"Mode: {vehicle.mode.name}\n"
            f"Armed: {vehicle.armed}\n"
            f"Location: {vehicle.location.global_frame}\n"
            f"Velocity: {vehicle.velocity}\n"
            f"Heading: {vehicle.heading}\n"
            f"{vehicle.battery}\n"
            f"GPS: {vehicle.gps_0}\n"
            f"Ground speed: {vehicle.groundspeed}\n"
            f"Airspeed: {vehicle.airspeed}\n"
            f"{vehicle.attitude}\n"
            f"Altitude: {vehicle.location.global_relative_frame.alt}\n\n"
        )
        print(output, end="\r")
        time.sleep(.1)  # Adjust the loop delay as needed.
except KeyboardInterrupt:
    print("\nExiting...")
