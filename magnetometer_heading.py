from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import math
import time

# Initialize
mpu = MPU9250(
    AK8963_ADDRESS,       # Magnetometer address
    MPU9050_ADDRESS_68,  # MPU address
    bus=1,
    gfs=GFS_250,                     # Gyroscope full scale
    afs=AFS_2G,                      # Accelerometer full scale
    mfs=AK8963_BIT_16,              # Magnetometer resolution
    mode=AK8963_MODE_C100HZ)        # Magnetometer mode (100Hz)

#mpu.calibrate()    # Optional: only needed once if you want offsets
mpu.configure()    # Apply settings

#def custom_magnetometer_calibration():


def calculate_heading(x, y):
    heading_rad = math.atan2(x, y)  # Not y, x — compass correction
    heading_deg = math.degrees(heading_rad) - 90
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

print("Reading heading...")
try:
    while True:
        mx, my, mz = mpu.readMagnetometerMaster()
        mx -= 23
        my -= 8
        heading = calculate_heading(mx, my)
        print(f"{mx}, {my}, {mz}")
        print(f"Heading: {heading:.2f}°")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopped.")

