import time
import math
import sys

from pymavlink import mavutil
import matplotlib.pyplot as plt

def latlon_to_local_xy(lat, lon, lat0, lon0):
    """
    Convert (lat, lon) in degrees to local XY in meters
    relative to (lat0, lon0) using an equirectangular approximation.
    """
    R = 6378137.0  # Earth radius (approx) in meters
    d_lat = math.radians(lat - lat0)
    d_lon = math.radians(lon - lon0)
    # Approximate x, y offsets
    x = d_lon * R * math.cos(math.radians((lat + lat0) / 2.0))
    y = d_lat * R
    return x, y

def main():
    # -------------------------------------------------------------------------
    # 1) Connect to Pixhawk via MAVLink
    # -------------------------------------------------------------------------
    connection_string = "/dev/ttyACM0"  # Adjust as needed
    baud_rate = 57600
    master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
    master.wait_heartbeat()
    print(f"Heartbeat received from system (system {master.target_system} component {master.target_component})")

    # -------------------------------------------------------------------------
    # 2) Data storage for console printing + plotting
    # -------------------------------------------------------------------------
    start_time = time.time()

    # Local origin for lat/lon
    lat0 = None
    lon0 = None

    # We'll zero altitude the first time we get a reading
    initial_alt = None

    # Latest measurements (initialize to 0)
    local_x = 0.0
    local_y = 0.0
    alt_m = 0.0
    airspeed = 0.0
    groundspeed = 0.0
    roll_rad = 0.0
    pitch_rad = 0.0
    yaw_rad = 0.0

    # For plotting
    times = []
    alt_list = []
    airspeed_list = []
    groundspeed_list = []
    roll_list = []
    pitch_list = []
    yaw_list = []
    local_x_list = []
    local_y_list = []

    # -------------------------------------------------------------------------
    # 3) Set up live plotting (matplotlib)
    # -------------------------------------------------------------------------
    plt.ion()  # interactive mode

    # (A) Timeseries Figure
    fig_timeseries, ax_timeseries = plt.subplots()
    line_alt, = ax_timeseries.plot([], [], label='Altitude (m)')
    line_airspeed, = ax_timeseries.plot([], [], label='Airspeed (m/s)')
    line_groundspeed, = ax_timeseries.plot([], [], label='Groundspeed (m/s)')
    line_roll, = ax_timeseries.plot([], [], label='Roll (deg)')
    line_pitch, = ax_timeseries.plot([], [], label='Pitch (deg)')
    line_yaw, = ax_timeseries.plot([], [], label='Yaw (deg)')
    ax_timeseries.legend()
    ax_timeseries.set_xlabel('Time (s)')
    ax_timeseries.set_ylabel('Value')
    ax_timeseries.set_title('Telemetry vs Time')

    # (B) Local 2D XY Position
    fig_location, ax_location = plt.subplots()
    line_location, = ax_location.plot([], [], marker='o', linestyle='-')
    ax_location.set_xlabel('Local X (m)')
    ax_location.set_ylabel('Local Y (m)')
    ax_location.set_title('2D Local Position')

    # -------------------------------------------------------------------------
    # 4) Main loop: read MAVLink, update console + plots
    # -------------------------------------------------------------------------
    last_print_time = time.time()

    try:
        while True:
            msg = master.recv_match(blocking=False)
            if msg is not None:
                msg_type = msg.get_type()
                if msg_type == "BAD_DATA":
                    continue

                # 4a) GLOBAL_POSITION_INT -> lat/lon/alt
                if msg_type == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt_raw = msg.alt / 1000.0  # mm -> m
                    if lat0 is None or lon0 is None:
                        lat0 = lat
                        lon0 = lon
                        print(f"Set local origin to lat={lat0:.7f}, lon={lon0:.7f}")
                    local_x, local_y = latlon_to_local_xy(lat, lon, lat0, lon0)

                    # Zero altitude if we haven't yet
                    if initial_alt is None:
                        initial_alt = alt_raw
                    alt_m = alt_raw - (initial_alt or 0.0)

                # 4b) VFR_HUD -> airspeed, groundspeed, alt (another alt source)
                elif msg_type == "VFR_HUD":
                    airspeed = msg.airspeed
                    groundspeed = msg.groundspeed
                    # This alt might differ from the GLOBAL_POSITION_INT alt
                    # If you prefer baro alt, you can do:
                    # if initial_alt is None:
                    #     initial_alt = msg.alt
                    # alt_m = msg.alt - (initial_alt or 0.0)

                # 4c) ATTITUDE -> pitch, roll, yaw (radians)
                elif msg_type == "ATTITUDE":
                    roll_rad = msg.roll
                    pitch_rad = msg.pitch
                    yaw_rad = msg.yaw

            # 4d) Update display
            current_time = time.time()
            if current_time - last_print_time > 0.05:  # 20 Hz
                t = current_time - start_time
                times.append(t)
                alt_list.append(alt_m)
                airspeed_list.append(airspeed)
                groundspeed_list.append(groundspeed)
                roll_list.append(roll_rad * 180.0 / math.pi)
                pitch_list.append(pitch_rad * 180.0 / math.pi)
                yaw_list.append(yaw_rad * 180.0 / math.pi)
                local_x_list.append(local_x)
                local_y_list.append(local_y)

                # Console output in one line
                console_str = (
                    f"R/P/Y=({roll_list[-1]:.1f},{pitch_list[-1]:.1f},{yaw_list[-1]:.1f}) "
                    f"| Alt={alt_m:.2f}m "
                    f"| Air={airspeed:.2f} GS={groundspeed:.2f} "
                    f"| Loc=({local_x:.1f},{local_y:.1f})"
                )
                sys.stdout.write(console_str.ljust(120) + "\r")
                sys.stdout.flush()

                # Update timeseries plot
                line_alt.set_data(times, alt_list)
                line_airspeed.set_data(times, airspeed_list)
                line_groundspeed.set_data(times, groundspeed_list)
                line_roll.set_data(times, roll_list)
                line_pitch.set_data(times, pitch_list)
                line_yaw.set_data(times, yaw_list)
                ax_timeseries.relim()
                ax_timeseries.autoscale_view()

                # Update 2D position plot
                line_location.set_data(local_x_list, local_y_list)
                ax_location.relim()
                ax_location.autoscale_view()

                plt.pause(0.001)
                last_print_time = current_time

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nExiting...")

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
