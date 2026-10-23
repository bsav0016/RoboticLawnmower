from evdev import InputDevice, categorize, ecodes, list_devices
from lawnmower.hardware.motors import GpioMotors
import time

# Setup motors
motors = GpioMotors()

def find_controller():
    keywords = ["zikway", "gamesir", "gamepad"]
    for path in list_devices():
        try:
            dev = InputDevice(path)
            name = dev.name.lower()
            if "keyboard" in name:
                continue
            caps = dev.capabilities()
            abs_axes = caps.get(ecodes.EV_ABS, [])
            if any(k in name for k in keywords) and len(abs_axes) >= 4:
                return dev
        except:
            continue
    return None

# Setup gamepad
gamepad = find_controller()
if gamepad is None:
    print("Controller not found!")
    motors.close()
    exit(1)

print(f"Listening to {gamepad.name} at {gamepad.path}...\n")

# Constants for mapping (may vary depending on your controller)
LEFT_Y = ecodes.ABS_Y    # forward/backward
LEFT_X = ecodes.ABS_X  # turning

def normalize_axis(value, center=128):
    # Normalize joystick axis to range -1 (full backward/left) to +1 (full forward/right)
    return (value - center) / (128)

def fuzzy_motor_speeds(x, y):
    """
    x: joystick X axis (left/right)
    y: joystick Y axis (forward/back)
    Returns: (left_motor_speed, right_motor_speed) in range -100 to 100
    """

    # Normalize inputs to -1..1
    norm_x = normalize_axis(x)
    norm_y = normalize_axis(y)

    # Apply deadzone to avoid noise around center
    deadzone = 0.1
    if abs(norm_x) < deadzone:
        norm_x = 0
    if abs(norm_y) < deadzone:
        norm_y = 0

    # Base speed
    base_speed = -norm_y * 100

    # Turning adjustment
    turn_adjust = norm_x * 100

    # Combine forward/backward and turning for each motor
    left_motor = base_speed + turn_adjust
    right_motor = base_speed - turn_adjust

    # Clamp motor speeds to -100 to 100
    left_motor = max(-100, min(100, left_motor))
    right_motor = max(-100, min(100, right_motor))

    return left_motor, right_motor


try:
    for event in gamepad.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == LEFT_X:
                x_val = event.value
            elif event.code == LEFT_Y:
                y_val = event.value

            if 'x_val' in locals() and 'y_val' in locals():
                left_speed, right_speed = fuzzy_motor_speeds(x_val, y_val)
                motors.set_speed(left_speed, right_speed)


        elif event.type == ecodes.EV_KEY:
            keyevent = categorize(event)
            if keyevent.keystate == 1:  # key down
                if keyevent.keycode == 'BTN_SOUTH':  # e.g., A button
                    motors.stop()
except KeyboardInterrupt:
    print("Stopping....")
finally:
    motors.close()
