import time

from lawnmower.hardware.motors import GpioMotors

# Bench smoke test. Uses full-scale speeds on both sides (no right-side trim), as before.
motors = GpioMotors(right_scale=1.0)

try:
    print("Motors forward")
    motors.set_speed(left_speed=90, right_speed=80)
    time.sleep(2)

    print("Motors backward")
    motors.set_speed(left_speed=-90, right_speed=-80)
    time.sleep(2)

    print("Stop motors")
    motors.stop()

finally:
    motors.close()
