import serial
import pynmea2
import math
import time
import RPi.GPIO as GPIO
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from create_waypoint_list import plan_field
from field_config import load_field
from lawnmower.geometry import calculate_bearing, distance_meters, heading_error, project_onto_row
from lawnmower.kalman import PositionFilter
from lawnmower.steering import heading_from_magnetometer, steer

# --- Lawnmower field corners (lat, lon) ---
FIELD = load_field()  # from field.json, or the corners in .env

# --- Navigation constants ---
STOP_DISTANCE_M = 0.5       # Waypoint reached when within 0.5 meters
RUNTIME_LIMIT_S = 180       # Stop after 3 minutes
FORWARD_SPEED = 100         # Motor speed when going straight (0-100)
TURN_SPEED = 80             # Motor speed for turning inner wheel
HEADING_THRESHOLD = 10      # Degrees of error before correcting course

# --- GPS setup ---
gps_port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

# --- IMU setup ---
mpu = MPU9250(
    AK8963_ADDRESS,
    MPU9050_ADDRESS_68,
    bus=1,
    gfs=GFS_250,
    afs=AFS_2G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ)
mpu.configure()

# --- Motor GPIO setup ---
GPIO.setmode(GPIO.BCM)
ENA = 18
IN1 = 17
IN2 = 27
ENB = 13
IN3 = 5
IN4 = 6

GPIO.setup([ENA, IN1, IN2, ENB, IN3, IN4], GPIO.OUT)
pwmA = GPIO.PWM(ENA, 1000)
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(0)
pwmB.start(0)


# --- Motor control ---
def motor_a_forward(speed=50):
    speed = speed * 0.9
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwmA.ChangeDutyCycle(speed)

def motor_a_backward(speed=50):
    speed = speed * 0.9
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwmA.ChangeDutyCycle(speed)

def motor_b_forward(speed=50):
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwmB.ChangeDutyCycle(speed)

def motor_b_backward(speed=50):
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwmB.ChangeDutyCycle(speed)

def stop_motor_a():
    pwmA.ChangeDutyCycle(0)
    GPIO.output([IN1, IN2], GPIO.LOW)

def stop_motor_b():
    pwmB.ChangeDutyCycle(0)
    GPIO.output([IN3, IN4], GPIO.LOW)

def stop_motors():
    stop_motor_a()
    stop_motor_b()

def set_motor_speed(left_speed, right_speed):
    # Motor A is wired to right side, Motor B to left side
    if right_speed > 0:
        motor_a_forward(right_speed)
    elif right_speed < 0:
        motor_a_backward(-right_speed)
    else:
        stop_motor_a()

    if left_speed > 0:
        motor_b_forward(left_speed)
    elif left_speed < 0:
        motor_b_backward(-left_speed)
    else:
        stop_motor_b()


# --- GPS ---
def read_gps():
    for _ in range(20):
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


# --- Accelerometer ---
def read_accelerometer_world(heading_deg):
    """Read accelerometer and rotate from body frame to world (East-North) frame.

    Assumes sensor x-axis = robot forward, y-axis = robot left.
    If the robot drifts in a consistent direction, swap or negate ax/ay here.
    readAccelerometerMaster() returns m/s^2; gravity (~9.81) is mostly on the
    z-axis when level and shows up as noise in x/y on hills — the Kalman
    process noise absorbs this."""
    ax, ay, az = mpu.readAccelerometerMaster()
    h = math.radians(heading_deg)
    # Rotate body (forward=x, left=y) → world (East, North)
    ax_east  =  ax * math.sin(h) - ay * math.cos(h)
    ay_north =  ax * math.cos(h) + ay * math.sin(h)
    return ax_east, ay_north


# --- Position filter ---
# GPS + accelerometer Kalman filter; see lawnmower/kalman.py for the state and tuning notes.
kf = PositionFilter()


# --- Generate waypoint path ---
WAYPOINTS = plan_field(FIELD)
waypoint_index = 0
TARGET_LAT, TARGET_LON = WAYPOINTS[waypoint_index]
prev_waypoint = None   # Tracks row start for path projection; set when a waypoint is reached

print(f"Path loaded: {len(WAYPOINTS)} waypoints.")
print(f"Navigating to waypoint 0: ({TARGET_LAT:.8f}, {TARGET_LON:.8f})")
start_time = time.time()
last_time   = start_time

try:
    while True:
        now     = time.time()
        dt      = max(now - last_time, 1e-3)
        last_time = now
        elapsed = now - start_time

        if elapsed >= RUNTIME_LIMIT_S:
            print("Time limit reached. Stopping.")
            break

        # --- Read sensors every loop ---
        heading = read_heading()
        ax_east, ay_north = read_accelerometer_world(heading)

        # --- Kalman predict (dead-reckon forward using accelerometer) ---
        if kf.initialized:
            kf.predict(ax_east, ay_north, dt)

        # --- GPS update (correct drift whenever a fix is available) ---
        gps = read_gps()
        if gps is not None:
            if not kf.initialized:
                kf.init(*gps)
            else:
                kf.update_gps(*gps)

        if not kf.initialized:
            print("Waiting for GPS fix...")
            time.sleep(0.5)
            continue

        # --- Best position estimate from Kalman filter ---
        cur_lat, cur_lon = kf.position()

        # --- Path projection: snap lateral position to the current row line ---
        # This prevents GPS noise from causing steering corrections while straight.
        # Not applied on the approach to the very first waypoint (no row yet).
        if prev_waypoint is not None:
            nav_lat, nav_lon = project_onto_row(
                cur_lat, cur_lon, prev_waypoint, (TARGET_LAT, TARGET_LON), kf.ref)
        else:
            nav_lat, nav_lon = cur_lat, cur_lon

        distance = distance_meters(nav_lat, nav_lon, TARGET_LAT, TARGET_LON)
        raw_str = f"({gps[0]:.6f}, {gps[1]:.6f})" if gps else "no fix"
        print(f"WP {waypoint_index}/{len(WAYPOINTS)-1} | "
              f"GPS: {raw_str} | KF: ({cur_lat:.6f}, {cur_lon:.6f}) | "
              f"Target: ({TARGET_LAT:.8f}, {TARGET_LON:.8f}) | "
              f"Dist: {distance:.1f}m | Elapsed: {elapsed:.1f}s")

        if distance <= STOP_DISTANCE_M:
            waypoint_index += 1
            if waypoint_index >= len(WAYPOINTS):
                print("All waypoints reached! Path complete.")
                break
            prev_waypoint = (TARGET_LAT, TARGET_LON)
            TARGET_LAT, TARGET_LON = WAYPOINTS[waypoint_index]
            print(f"Waypoint reached. Moving to waypoint {waypoint_index}/{len(WAYPOINTS)-1}: "
                  f"({TARGET_LAT:.8f}, {TARGET_LON:.8f})")
            continue

        bearing = calculate_bearing(nav_lat, nav_lon, TARGET_LAT, TARGET_LON)
        error   = heading_error(heading, bearing)
        print(f"Heading: {heading:.1f}° | Target bearing: {bearing:.1f}° | Error: {error:.1f}°")

        set_motor_speed(*steer(error, FORWARD_SPEED, TURN_SPEED, HEADING_THRESHOLD))

except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    stop_motors()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
    gps_port.close()
    print("Shutdown complete.")
