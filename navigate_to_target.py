import serial
import pynmea2
import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from lawnmower.geometry import calculate_bearing, distance_meters, heading_error
from lawnmower.hardware.motors import GpioMotors
from lawnmower.steering import heading_from_magnetometer, steer

# --- Target GPS Coordinate ---
TARGET_LAT = 37.123456
TARGET_LON = -122.123456

# --- Navigation constants ---
STOP_DISTANCE_M = 1.0       # Stop when within 1 meter of target
RUNTIME_LIMIT_S = 30        # Stop after 30 seconds
FORWARD_SPEED = 100         # Motor speed when going straight (0-100)
TURN_SPEED = 80             # Motor speed for turning inner wheel
HEADING_THRESHOLD = 10      # Degrees of error before correcting course

# --- GPS setup ---
gps_port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

# --- Magnetometer setup ---
mpu = MPU9250(
    AK8963_ADDRESS,
    MPU9050_ADDRESS_68,
    bus=1,
    gfs=GFS_250,
    afs=AFS_2G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ)
mpu.configure()

# --- Motors ---
motors = GpioMotors()


# --- GPS ---
def read_gps():
    for _ in range(20):  # Try up to 20 lines before giving up
        try:
            line = gps_port.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                if msg.latitude:
                    return msg.latitude, msg.longitude
        except pynmea2.ParseError:
            continue
    return None


# --- Magnetometer ---
def read_heading():
    mx, my, mz = mpu.readMagnetometerMaster()
    return heading_from_magnetometer(mx, my)


# --- Main loop ---
print(f"Navigating to ({TARGET_LAT}, {TARGET_LON})...")
start_time = time.time()

try:
    while True:
        elapsed = time.time() - start_time
        if elapsed >= RUNTIME_LIMIT_S:
            print("Time limit reached. Stopping.")
            break

        position = read_gps()
        if position is None:
            print("Waiting for GPS fix...")
            time.sleep(0.5)
            continue

        cur_lat, cur_lon = position
        distance = distance_meters(cur_lat, cur_lon, TARGET_LAT, TARGET_LON)
        print(f"GPS: ({cur_lat:.8f}, {cur_lon:.8f}) | Target: ({TARGET_LAT:.8f}, {TARGET_LON:.8f}) | Distance: {distance:.1f}m | Elapsed: {elapsed:.1f}s")

        if distance <= STOP_DISTANCE_M:
            print("Target reached!")
            break

        heading = read_heading()
        bearing = calculate_bearing(cur_lat, cur_lon, TARGET_LAT, TARGET_LON)
        error = heading_error(heading, bearing)
        print(f"Heading: {heading:.1f}° | Target bearing: {bearing:.1f}° | Error: {error:.1f}°")

        motors.set_speed(*steer(error, FORWARD_SPEED, TURN_SPEED, HEADING_THRESHOLD))

except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    motors.close()
    gps_port.close()
    print("Shutdown complete.")
