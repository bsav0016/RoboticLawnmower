import serial
import pynmea2
import math
import time
import RPi.GPIO as GPIO
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from create_waypoint_list import plan_field
from field_config import load_field
from lawnmower.mission import Mission, MissionConfig, WAITING_FOR_FIX
from lawnmower.steering import heading_from_magnetometer

# --- Lawnmower field corners (lat, lon) ---
FIELD = load_field()  # from field.json, or the corners in .env

# --- Navigation settings (the decisions themselves are in lawnmower/mission.py) ---
CONFIG = MissionConfig(
    stop_distance_m=0.5,        # Waypoint reached when within 0.5 meters
    runtime_limit_s=180,        # Stop after 3 minutes
    forward_speed=100,          # Motor speed when going straight (0-100)
    turn_speed=80,              # Motor speed for turning inner wheel
    heading_threshold_deg=10,   # Degrees of error before correcting course
)

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


# --- Generate waypoint path ---
WAYPOINTS = plan_field(FIELD)
start_time = time.time()
mission = Mission(WAYPOINTS, CONFIG, now=start_time)   # holds the position filter and the current waypoint
for line in mission.start_messages():
    print(line)

try:
    while True:
        now = time.time()

        # --- Read sensors every loop ---
        heading = read_heading()
        ax_east, ay_north = read_accelerometer_world(heading)
        gps = read_gps()

        # --- Decide: Kalman filter, waypoint advance, path projection and steering ---
        command = mission.step(now, heading, ax_east, ay_north, gps)
        for line in command.messages:
            print(line)
        if command.done:
            break
        if command.reason == WAITING_FOR_FIX:
            time.sleep(0.5)
            continue
        if command.left is not None:
            set_motor_speed(command.left, command.right)

except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    stop_motors()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
    gps_port.close()
    print("Shutdown complete.")
