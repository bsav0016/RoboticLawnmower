"""Heading and steering decisions, as pure functions."""
import math


def heading_from_magnetometer(mx, my, offset_x=23, offset_y=8):
    """Compass heading in degrees (0-360) from magnetometer x/y after removing the hard-iron offsets.

    The defaults are the offsets measured on the prototype. The fixed -90 degree term assumes the
    sensor is mounted level with its axes rotated a quarter turn from the robot's forward direction."""
    mx -= offset_x
    my -= offset_y
    heading_deg = math.degrees(math.atan2(mx, my)) - 90
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg


def steer(error, forward_speed, turn_speed, threshold_deg):
    """Return (left_speed, right_speed) for a heading error in degrees (positive = target is to the right)."""
    if error > threshold_deg:
        return forward_speed, turn_speed   # turn right: slow the right wheel
    if error < -threshold_deg:
        return turn_speed, forward_speed   # turn left: slow the left wheel
    return forward_speed, forward_speed
