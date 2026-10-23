"""Position filter: a 4-state Kalman filter that fuses GPS fixes with accelerometer dead reckoning.

State: [x, y, vx, vy] in local meters (East-North from the origin of the first GPS fix).
Process input: accelerometer (ax_east, ay_north) in m/s^2.
Measurement: GPS position.

Tune the noise values if the filter is too sluggish (raise gps_std_m) or too noisy (lower it),
or if dead reckoning drifts fast (raise process_vel)."""
import numpy as np

from .geometry import to_gps, to_meters

GPS_STD_M = 3.0       # GPS 1-sigma position noise (meters)
PROCESS_POS = 0.05    # Position process noise std dev (m per step)
PROCESS_VEL = 0.3     # Velocity process noise std dev (m/s per step)


class PositionFilter:
    def __init__(self, gps_std_m=GPS_STD_M, process_pos=PROCESS_POS, process_vel=PROCESS_VEL):
        self.gps_std_m = gps_std_m
        self.process_pos = process_pos
        self.process_vel = process_vel
        self.ref = None      # (lat, lon) origin of the local frame; None until the first fix
        self.state = None    # np.array [x, y, vx, vy]
        self.P = None        # 4x4 covariance matrix

    @property
    def initialized(self):
        return self.ref is not None

    def init(self, lat, lon):
        """Start the filter at a GPS fix, which becomes the origin of the local frame."""
        self.ref = (lat, lon)
        self.state = np.zeros(4)
        self.P = np.diag([self.gps_std_m ** 2, self.gps_std_m ** 2, 1.0, 1.0])

    def predict(self, ax_east, ay_north, dt):
        """Dead-reckon forward by dt seconds using the accelerometer reading."""
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        B = np.array([[0.5 * dt ** 2, 0],
                      [0, 0.5 * dt ** 2],
                      [dt, 0],
                      [0, dt]], dtype=float)
        Q = np.diag([self.process_pos ** 2, self.process_pos ** 2,
                     self.process_vel ** 2, self.process_vel ** 2])
        self.state = F @ self.state + B @ np.array([ax_east, ay_north])
        self.P = F @ self.P @ F.T + Q

    def update_gps(self, lat, lon):
        """Correct the estimate with a GPS fix."""
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=float)
        R = np.eye(2) * self.gps_std_m ** 2
        z = np.array(to_meters(lat, lon, *self.ref))
        innov = z - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ innov
        self.P = (np.eye(4) - K @ H) @ self.P

    def position(self):
        """Current best (lat, lon) estimate."""
        return to_gps(self.state[0], self.state[1], *self.ref)

    def position_std_m(self):
        """Rough 1-sigma position uncertainty in meters (RMS of the x and y variances)."""
        return float(np.sqrt((self.P[0, 0] + self.P[1, 1]) / 2))
