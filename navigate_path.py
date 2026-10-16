import serial
import pynmea2
import math
import time
import numpy as np
import RPi.GPIO as GPIO
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from create_waypoint_list import create_waypoint_list
from field_config import load_corners

# --- Lawnmower field corners (lat, lon) ---
CORNER_A, CORNER_B, CORNER_C, CORNER_D = load_corners()  # from .env

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
    mx -= 23
    my -= 8
    heading_rad = math.atan2(mx, my)
    heading_deg = math.degrees(heading_rad) - 90
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg


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


# --- Kalman filter ---
# State: [x, y, vx, vy] in local meters (East-North from _kf_ref origin)
# Process input: accelerometer (ax_east, ay_north)
# Measurement: GPS position (x, y)
#
# Tune these if the filter is too sluggish (raise GPS_STD_M) or too noisy
# (lower GPS_STD_M), or if dead-reckoning drifts fast (raise KF_PROCESS_VEL).
GPS_STD_M      = 3.0   # GPS 1-sigma position noise (meters)
KF_PROCESS_POS = 0.05  # Position process noise std dev (m per step)
KF_PROCESS_VEL = 0.3   # Velocity process noise std dev (m/s per step)

_kf_initialized = False
_kf_state = None   # np.array [x, y, vx, vy]
_kf_P     = None   # 4×4 covariance matrix
_kf_ref   = None   # (lat, lon) origin of local coordinate system


def _to_local(lat, lon):
    ref_lat, ref_lon = _kf_ref
    y = (lat - ref_lat) * 111111.0
    x = (lon - ref_lon) * 111111.0 * math.cos(math.radians(ref_lat))
    return x, y

def _to_latlon(x, y):
    ref_lat, ref_lon = _kf_ref
    lat = ref_lat + y / 111111.0
    lon = ref_lon + x / (111111.0 * math.cos(math.radians(ref_lat)))
    return lat, lon

def kf_init(lat, lon):
    global _kf_initialized, _kf_state, _kf_P, _kf_ref
    _kf_ref   = (lat, lon)
    _kf_state = np.zeros(4)
    _kf_P     = np.diag([GPS_STD_M**2, GPS_STD_M**2, 1.0, 1.0])
    _kf_initialized = True

def kf_predict(ax_east, ay_north, dt):
    global _kf_state, _kf_P
    F = np.array([[1, 0, dt,  0],
                  [0, 1,  0, dt],
                  [0, 0,  1,  0],
                  [0, 0,  0,  1]], dtype=float)
    B = np.array([[0.5*dt**2,        0],
                  [0,         0.5*dt**2],
                  [dt,                0],
                  [0,                dt]], dtype=float)
    Q = np.diag([KF_PROCESS_POS**2, KF_PROCESS_POS**2,
                 KF_PROCESS_VEL**2, KF_PROCESS_VEL**2])
    _kf_state = F @ _kf_state + B @ np.array([ax_east, ay_north])
    _kf_P     = F @ _kf_P @ F.T + Q

def kf_update_gps(lat, lon):
    global _kf_state, _kf_P
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]], dtype=float)
    R = np.eye(2) * GPS_STD_M**2
    z = np.array(_to_local(lat, lon))
    innov = z - H @ _kf_state
    S = H @ _kf_P @ H.T + R
    K = _kf_P @ H.T @ np.linalg.inv(S)
    _kf_state = _kf_state + K @ innov
    _kf_P     = (np.eye(4) - K @ H) @ _kf_P

def kf_position():
    return _to_latlon(_kf_state[0], _kf_state[1])


# --- Navigation math ---
def calculate_bearing(cur_lat, cur_lon, tgt_lat, tgt_lon):
    lat1 = math.radians(cur_lat)
    lat2 = math.radians(tgt_lat)
    dlon = math.radians(tgt_lon - cur_lon)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360

def heading_error(current_heading, target_bearing):
    error = target_bearing - current_heading
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

def distance_meters(cur_lat, cur_lon, tgt_lat, tgt_lon):
    R = 6371000
    lat1, lat2 = math.radians(cur_lat), math.radians(tgt_lat)
    dlat = math.radians(tgt_lat - cur_lat)
    dlon = math.radians(tgt_lon - cur_lon)
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return R * 2 * math.asin(math.sqrt(a))

def project_onto_row(cur_lat, cur_lon, row_start, row_end):
    """Project the current position onto the current row line segment.
    This eliminates lateral drift during straight runs — the robot navigates
    as if it's exactly on the line, so small sideways GPS errors don't cause
    unnecessary steering corrections."""
    px, py = _to_local(cur_lat, cur_lon)
    ax, ay = _to_local(*row_start)
    bx, by = _to_local(*row_end)
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx*dx + dy*dy
    if seg_len_sq < 1e-10:
        return cur_lat, cur_lon
    t = max(0.0, min(1.0, ((px - ax)*dx + (py - ay)*dy) / seg_len_sq))
    return _to_latlon(ax + t*dx, ay + t*dy)


# --- Generate waypoint path ---
WAYPOINTS = create_waypoint_list(CORNER_A, CORNER_B, CORNER_C, CORNER_D)
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
        if _kf_initialized:
            kf_predict(ax_east, ay_north, dt)

        # --- GPS update (correct drift whenever a fix is available) ---
        gps = read_gps()
        if gps is not None:
            if not _kf_initialized:
                kf_init(*gps)
            else:
                kf_update_gps(*gps)

        if not _kf_initialized:
            print("Waiting for GPS fix...")
            time.sleep(0.5)
            continue

        # --- Best position estimate from Kalman filter ---
        cur_lat, cur_lon = kf_position()

        # --- Path projection: snap lateral position to the current row line ---
        # This prevents GPS noise from causing steering corrections while straight.
        # Not applied on the approach to the very first waypoint (no row yet).
        if prev_waypoint is not None:
            nav_lat, nav_lon = project_onto_row(
                cur_lat, cur_lon, prev_waypoint, (TARGET_LAT, TARGET_LON))
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

        if error > HEADING_THRESHOLD:
            set_motor_speed(FORWARD_SPEED, TURN_SPEED)
        elif error < -HEADING_THRESHOLD:
            set_motor_speed(TURN_SPEED, FORWARD_SPEED)
        else:
            set_motor_speed(FORWARD_SPEED, FORWARD_SPEED)

except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    stop_motors()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
    gps_port.close()
    print("Shutdown complete.")
