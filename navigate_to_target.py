import serial
import pynmea2
import math
import time
import RPi.GPIO as GPIO
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

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
    mx -= 23
    my -= 8
    heading_rad = math.atan2(mx, my)
    heading_deg = math.degrees(heading_rad) - 90
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg


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
    # Normalize to -180..180
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

def distance_meters(cur_lat, cur_lon, tgt_lat, tgt_lon):
    R = 6371000  # Earth radius in meters
    lat1, lat2 = math.radians(cur_lat), math.radians(tgt_lat)
    dlat = math.radians(tgt_lat - cur_lat)
    dlon = math.radians(tgt_lon - cur_lon)
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return R * 2 * math.asin(math.sqrt(a))


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

        if error > HEADING_THRESHOLD:
            # Turn right
            set_motor_speed(FORWARD_SPEED, TURN_SPEED)
        elif error < -HEADING_THRESHOLD:
            # Turn left
            set_motor_speed(TURN_SPEED, FORWARD_SPEED)
        else:
            # Go straight
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
