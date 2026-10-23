"""Tests for the motor drivers, using a fake GPIO module (no Raspberry Pi needed)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lawnmower.hardware import motors as m  # noqa: E402


class FakePwm:
    def __init__(self, pin):
        self.pin = pin
        self.duty = None
        self.started = False
        self.stopped = False

    def start(self, duty):
        self.started = True
        self.duty = duty

    def ChangeDutyCycle(self, duty):
        self.duty = duty

    def stop(self):
        self.stopped = True


class FakeGpio:
    BCM, OUT, HIGH, LOW = "BCM", "OUT", 1, 0

    def __init__(self):
        self.pins = {}
        self.pwms = {}
        self.mode = None
        self.configured = None
        self.cleaned = False

    def setmode(self, mode):
        self.mode = mode

    def setup(self, pins, direction):
        self.configured = (list(pins), direction)

    def output(self, pins, value):
        for pin in pins if isinstance(pins, (list, tuple)) else [pins]:
            self.pins[pin] = value

    def PWM(self, pin, hz):
        self.pwms[pin] = FakePwm(pin)
        self.pwms[pin].hz = hz
        return self.pwms[pin]

    def cleanup(self):
        self.cleaned = True


def make(**kwargs):
    gpio = FakeGpio()
    return gpio, m.GpioMotors(gpio=gpio, **kwargs)


class GpioSetupTests(unittest.TestCase):
    def test_configures_the_l298n_pins_and_pwm(self):
        gpio, _ = make()
        self.assertEqual(gpio.mode, "BCM")
        self.assertEqual(gpio.configured, ([18, 17, 27, 13, 5, 6], "OUT"))
        self.assertEqual({p: (w.hz, w.started, w.duty) for p, w in gpio.pwms.items()},
                         {18: (1000, True, 0), 13: (1000, True, 0)})


class GpioDriveTests(unittest.TestCase):
    def test_forward_on_both_sides(self):
        gpio, mot = make()
        mot.set_speed(left_speed=60, right_speed=100)
        # motor A (right): IN1 high, IN2 low, duty scaled by 0.9
        self.assertEqual((gpio.pins[m.IN1], gpio.pins[m.IN2]), (1, 0))
        self.assertAlmostEqual(gpio.pwms[m.ENA].duty, 90)
        # motor B (left): IN3 low, IN4 high, duty as given
        self.assertEqual((gpio.pins[m.IN3], gpio.pins[m.IN4]), (0, 1))
        self.assertEqual(gpio.pwms[m.ENB].duty, 60)

    def test_reverse_on_both_sides(self):
        gpio, mot = make()
        mot.set_speed(left_speed=-50, right_speed=-40)
        self.assertEqual((gpio.pins[m.IN1], gpio.pins[m.IN2]), (0, 1))
        self.assertAlmostEqual(gpio.pwms[m.ENA].duty, 36)
        self.assertEqual((gpio.pins[m.IN3], gpio.pins[m.IN4]), (1, 0))
        self.assertEqual(gpio.pwms[m.ENB].duty, 50)

    def test_zero_stops_only_that_side(self):
        gpio, mot = make()
        mot.set_speed(left_speed=0, right_speed=70)
        self.assertEqual((gpio.pins[m.IN3], gpio.pins[m.IN4]), (0, 0))
        self.assertEqual(gpio.pwms[m.ENB].duty, 0)
        self.assertEqual((gpio.pins[m.IN1], gpio.pins[m.IN2]), (1, 0))
        self.assertGreater(gpio.pwms[m.ENA].duty, 0)

    def test_stop_zeroes_every_pin_and_duty(self):
        gpio, mot = make()
        mot.set_speed(100, 100)
        mot.stop()
        self.assertEqual([gpio.pins[p] for p in (m.IN1, m.IN2, m.IN3, m.IN4)], [0, 0, 0, 0])
        self.assertEqual((gpio.pwms[m.ENA].duty, gpio.pwms[m.ENB].duty), (0, 0))

    def test_right_scale_can_be_disabled_for_the_bench_test(self):
        gpio, mot = make(right_scale=1.0)
        mot.set_speed(left_speed=90, right_speed=80)
        self.assertEqual(gpio.pwms[m.ENA].duty, 80)
        self.assertEqual(gpio.pwms[m.ENB].duty, 90)

    def test_close_stops_then_releases_the_hardware(self):
        gpio, mot = make()
        mot.set_speed(100, 100)
        mot.close()
        self.assertEqual((gpio.pwms[m.ENA].duty, gpio.pwms[m.ENB].duty), (0, 0))
        self.assertTrue(gpio.pwms[m.ENA].stopped and gpio.pwms[m.ENB].stopped)
        self.assertTrue(gpio.cleaned)


class SimMotorsTests(unittest.TestCase):
    def test_records_commands_and_current_speeds(self):
        sim = m.SimMotors()
        sim.set_speed(100, 80)
        sim.set_speed(-20, 30)
        self.assertEqual(sim.speeds, (-20, 30))
        self.assertEqual(sim.history, [(100, 80), (-20, 30)])

    def test_stop_and_close(self):
        sim = m.SimMotors()
        sim.set_speed(100, 100)
        sim.close()
        self.assertEqual(sim.speeds, (0, 0))
        self.assertTrue(sim.closed)

    def test_is_a_motors_interface(self):
        self.assertIsInstance(m.SimMotors(), m.Motors)
        self.assertIsInstance(make()[1], m.Motors)


if __name__ == "__main__":
    unittest.main()
