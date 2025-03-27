from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'  # Replace with your actual port

lidar = RPLidar(PORT_NAME, baudrate=256000)

for i, scan in enumerate(lidar.iter_scans()):
    print(f'Scan {i}: {scan}')
    if i > 5:
        break

lidar.stop()
lidar.disconnect()
