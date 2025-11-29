from TercioLab import Bridge, IMU
import time

bridge = Bridge()
bridge.open()

imu = IMU(bridge, 3)

while True:
    print(imu.get_roll())
    time.sleep(0.25)