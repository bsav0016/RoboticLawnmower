"""Motor drivers for the L298N board: one real driver and one simulated driver behind the same interface.

Wiring: motor A (ENA/IN1/IN2) is the right side, motor B (ENB/IN3/IN4) is the left side. Speeds are
-100 (full reverse) to 100 (full forward); 0 stops that side."""

ENA, IN1, IN2 = 18, 17, 27   # motor A: right side
ENB, IN3, IN4 = 13, 5, 6     # motor B: left side
PWM_HZ = 1000
RIGHT_SCALE = 0.9            # motor A runs a little faster than B, so it is scaled down


class Motors:
    """Interface shared by the real and simulated drivers."""

    def set_speed(self, left_speed, right_speed):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def close(self):
        """Stop the motors and release the hardware."""
        raise NotImplementedError


class GpioMotors(Motors):
    """Drives the L298N from the Pi's GPIO pins. `gpio` is injectable for tests; by default it is RPi.GPIO."""

    def __init__(self, right_scale=RIGHT_SCALE, gpio=None):
        if gpio is None:
            import RPi.GPIO as gpio
        self.gpio = gpio
        self.right_scale = right_scale
        gpio.setmode(gpio.BCM)
        gpio.setup([ENA, IN1, IN2, ENB, IN3, IN4], gpio.OUT)
        self.pwm_a = gpio.PWM(ENA, PWM_HZ)
        self.pwm_b = gpio.PWM(ENB, PWM_HZ)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def _drive_a(self, speed):
        gpio = self.gpio
        if speed > 0:
            gpio.output(IN1, gpio.HIGH)
            gpio.output(IN2, gpio.LOW)
            self.pwm_a.ChangeDutyCycle(speed * self.right_scale)
        elif speed < 0:
            gpio.output(IN1, gpio.LOW)
            gpio.output(IN2, gpio.HIGH)
            self.pwm_a.ChangeDutyCycle(-speed * self.right_scale)
        else:
            self.pwm_a.ChangeDutyCycle(0)
            gpio.output([IN1, IN2], gpio.LOW)

    def _drive_b(self, speed):
        gpio = self.gpio
        if speed > 0:
            gpio.output(IN3, gpio.LOW)
            gpio.output(IN4, gpio.HIGH)
            self.pwm_b.ChangeDutyCycle(speed)
        elif speed < 0:
            gpio.output(IN3, gpio.HIGH)
            gpio.output(IN4, gpio.LOW)
            self.pwm_b.ChangeDutyCycle(-speed)
        else:
            self.pwm_b.ChangeDutyCycle(0)
            gpio.output([IN3, IN4], gpio.LOW)

    def set_speed(self, left_speed, right_speed):
        self._drive_a(right_speed)
        self._drive_b(left_speed)

    def stop(self):
        self.set_speed(0, 0)

    def close(self):
        self.stop()
        self.pwm_a.stop()
        self.pwm_b.stop()
        self.gpio.cleanup()


class SimMotors(Motors):
    """Records the commands instead of driving anything. `speeds` is the current (left, right) pair."""

    def __init__(self):
        self.speeds = (0, 0)
        self.history = []
        self.closed = False

    def set_speed(self, left_speed, right_speed):
        self.speeds = (left_speed, right_speed)
        self.history.append(self.speeds)

    def stop(self):
        self.set_speed(0, 0)

    def close(self):
        self.stop()
        self.closed = True
