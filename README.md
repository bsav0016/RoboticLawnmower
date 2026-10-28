# Robotic Lawnmower

A Raspberry Pi 4 rover that drives itself over a GPS-bounded rectangle in a boustrophedon ("lawnmower") pattern. It fuses GPS, magnetometer and accelerometer readings with a Kalman filter and steers two DC motors toward each waypoint. This is the prototype platform; the long-term goal is a full-size mower.

<p>
<img src="docs/images/cart-overview.jpg" alt="The rover prototype: Raspberry Pi 4, GPS module, magnetometer board, L298N motor driver and two drive wheels on an acrylic chassis" height="420">
<img src="docs/images/cart-side.jpg" alt="Side view of the rover showing the Pi GPIO wiring, GPS module and USB controller dongle" height="420">
</p>

![Wiring diagram: LiPo battery feeding a buck converter and motor controller, with the Pi, GPS and IMU](docs/images/wiring-diagram.png)

## How it works

1. **Coverage planning** – [create_waypoint_list.py](create_waypoint_list.py) takes four corner coordinates, converts them to a local meter grid (flat-earth approximation, fine under ~1 km), and generates evenly spaced parallel passes. The corners are checked first ([lawnmower/field.py](lawnmower/field.py)): a field wider than 500 m, corners on top of each other, a line instead of an area or a plan of more than 5,000 waypoints stops with a message instead of a plan. The result is written to [waypoints.py](waypoints.py).
2. **Sensing** – a NEO-6M GPS (UART, NMEA via `pynmea2`) gives position; an MPU-9250 gives magnetometer heading and acceleration over I²C.
3. **State estimation** – [navigate_path.py](navigate_path.py) runs a 4-state Kalman filter (`x, y, vx, vy`). The accelerometer, rotated into the East/North frame using the compass heading, drives the predict step, and GPS fixes drive the update step. GPS alone was too noisy to steer on, and the robot has no wheel encoders, so the filter bridges the gaps.
4. **Path following** – the position estimate is projected onto the current row so sideways GPS error does not cause needless steering. Heading error against the target bearing selects straight driving or a differential-speed turn.
5. **Actuation** – PWM on an L298N motor driver (BCM pins 18/17/27 for motor A, 13/5/6 for motor B).

## Hardware

| Function | Part |
| --- | --- |
| Computer | Raspberry Pi 4 |
| Power | 7.4 V 2S LiPo, MP1584EN buck converter to 5 V for the Pi |
| Motor driver | L298N |
| Drive | 2 DC gear motors, caster wheel, smart-robot chassis kit |
| GPS | GY-NEO6MV2 (u-blox NEO-6M) |
| Compass / IMU | MPU-9250 |
| Manual control | GameSir Nova Lite Bluetooth controller |

The wiring diagram source is [Wiring Diagram.pptx](Wiring%20Diagram.pptx); the parts list and project plan are in [Overview.docx](Overview.docx).

## Scripts

| Script | Purpose |
| --- | --- |
| [navigate_path.py](navigate_path.py) | Full autonomous run: waypoint generation, Kalman filter, path following |
| [navigate_to_target.py](navigate_to_target.py) | Drive to a single GPS coordinate (earlier milestone) |
| [create_waypoint_list.py](create_waypoint_list.py) | Generate lawnmower passes from four corners |
| [control_motors.py](control_motors.py) | Drive with a Bluetooth gamepad |
| [basic_motor_control.py](basic_motor_control.py) | Motor driver smoke test |
| [calibrate_magnetometer.py](calibrate_magnetometer.py) | Collect samples for hard-iron offsets (data in the `*calibration.txt` files and [Magnetometer Offsets.xlsx](Magnetometer%20Offsets.xlsx)) |
| [magnetometer_heading.py](magnetometer_heading.py) | Print compass heading |
| [read_gps.py](read_gps.py) | Print raw GPS fixes |
| [display_controller_input.py](display_controller_input.py), [find_controller_path.py](find_controller_path.py) | Gamepad discovery and debugging |

The navigation maths shared by the scripts lives in the [lawnmower/](lawnmower) package (`geometry.py`: bearing, distance, local frame and row projection; `steering.py`: compass heading and the turn decision; `kalman.py`: the GPS + accelerometer position filter). It imports no Pi hardware libraries, so it runs and is tested anywhere.

## Running it

On a Raspberry Pi with I²C and the serial port enabled:

```bash
pip install pyserial pynmea2 numpy RPi.GPIO mpu9250-jmdev
cp .env.example .env   # then set CORNER_A..CORNER_D to your field's corners ("lat,lon")
python navigate_path.py
```

Field coordinates live in `.env`, which is gitignored.

The run stops after `RUNTIME_LIMIT_S` seconds as a safety cutoff.

### Tests

```bash
python -m unittest discover -s tests
```

The tests need no Pi, only `numpy`. They cover the waypoint planner (coordinate conversion, corner ordering, pass count, spacing, coverage and direction for wide and tall fields), the `.env` corner loading, and the `lawnmower/` navigation maths (bearing wraparound, heading error at ±180°, distance, row projection and clamping, compass heading, steering decision), and the Kalman filter (convergence on a fixed target, noise smoothing, dead-reckoning kinematics, uncertainty growth and shrinkage). The sensor reading and motor code still only run on the robot and are not tested; GitHub Actions just checks that every script still compiles, then runs these tests. Dependabot opens weekly pull requests to update the pinned GitHub Actions versions.

## Milestones

- [x] Drive with a remote controller
- [x] Drive to a single GPS coordinate
- [~] Cover a GPS-outlined sector (Kalman filtering and path projection in progress)
- [ ] Traverse a sector with obstacles
- [ ] Scale up the chassis
- [ ] Add cutting

## Lessons so far

- Consumer GPS wanders by several meters, so raw fixes make the robot hunt. A simple EMA helped but did not fix it; fusing the IMU in a Kalman filter did more.
- Without wheel encoders, speed is unknown on hills and rough ground. Accelerometer integration drifts, so GPS has to remain the long-term anchor.
- One mistyped digit in a corner longitude (`-86` for `-85`) made the planner generate about 175,000 waypoints for a field roughly 87 km wide. The planner now refuses a field like that (and other impossible shapes) with a message that says what to check; the field limits are in [lawnmower/field.py](lawnmower/field.py).

## License

MIT. See [LICENSE](LICENSE).
