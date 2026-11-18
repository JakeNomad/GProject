"""Constant-velocity Kalman filter and a simple multi-object tracker.

State x = [px, py, vx, vy] in base_link. Pure numpy, no ROS dependency, so it
can be unit-tested on its own.
"""
import numpy as np


class ConstantVelocityKF:

    def __init__(self, px, py, pos_var=1.0, vel_var=10.0, accel_noise=2.0):
        self.x = np.array([px, py, 0.0, 0.0])
        self.P = np.diag([pos_var, pos_var, vel_var, vel_var])
        self.q = accel_noise ** 2

    def predict(self, dt):
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        # discrete white-noise acceleration model
        g = np.array([[0.5 * dt ** 2, 0.0], [0.0, 0.5 * dt ** 2], [dt, 0.0], [0.0, dt]])
        Q = g @ g.T * self.q
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z, H, R):
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    def update_position(self, px, py, var):
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        self.update(np.array([px, py]), H, np.eye(2) * var)

    def update_radial_velocity(self, vr, var):
        """Radar Doppler: vr = (p . v) / |p|  (linearised around the current state)."""
        px, py = self.x[0], self.x[1]
        r = max(np.hypot(px, py), 1e-3)
        H = np.array([[0.0, 0.0, px / r, py / r]])
        self.update(np.array([vr]), H, np.array([[var]]))

    def mahalanobis_sq(self, px, py, var):
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        y = np.array([px, py]) - H @ self.x
        S = H @ self.P @ H.T + np.eye(2) * var
        return float(y @ np.linalg.solve(S, y))


class Track:
    _next_id = 1

    def __init__(self, px, py, label='unknown'):
        self.id = Track._next_id
        Track._next_id += 1
        self.kf = ConstantVelocityKF(px, py)
        self.label = label
        self.confidence = 0.3
        self.size = (1.0, 1.0, 1.0)
        self.hits = 1
        self.misses = 0


class MultiObjectTracker:
    """Global-nearest-neighbour data association with chi-square gating."""

    GATE = 9.21  # chi2(2 dof, 99%)

    def __init__(self, lidar_var=0.2, radar_pos_var=1.0, radar_vel_var=0.5, max_misses=5):
        self.tracks = []
        self.lidar_var = lidar_var
        self.radar_pos_var = radar_pos_var
        self.radar_vel_var = radar_vel_var
        self.max_misses = max_misses

    def predict(self, dt):
        for t in self.tracks:
            t.kf.predict(dt)

    def _associate(self, positions, var):
        """Greedy assignment by increasing Mahalanobis distance. Returns {det_idx: track}."""
        pairs = sorted(
            (t.kf.mahalanobis_sq(px, py, var), i, t)
            for i, (px, py) in enumerate(positions) for t in self.tracks)
        used_d, used_t, out = set(), set(), {}
        for d2, i, t in pairs:
            if d2 > self.GATE or i in used_d or t.id in used_t:
                continue
            used_d.add(i)
            used_t.add(t.id)
            out[i] = t
        return out

    def update_lidar(self, detections):
        """detections: list of (px, py, (sx, sy, sz)). Creates tracks for unmatched ones."""
        matched = self._associate([(d[0], d[1]) for d in detections], self.lidar_var)
        seen = set()
        for i, (px, py, size) in enumerate(detections):
            t = matched.get(i)
            if t is None:
                t = Track(px, py)
                self.tracks.append(t)
            else:
                t.kf.update_position(px, py, self.lidar_var)
                t.hits += 1
                t.confidence = min(1.0, t.confidence + 0.1)
            t.size = size
            t.misses = 0
            seen.add(t.id)
        for t in self.tracks:
            if t.id not in seen:
                t.misses += 1
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

    def update_radar(self, detections):
        """detections: list of (px, py, radial_velocity). Only refines existing tracks."""
        matched = self._associate([(d[0], d[1]) for d in detections], self.radar_pos_var)
        for i, t in matched.items():
            px, py, vr = detections[i]
            t.kf.update_position(px, py, self.radar_pos_var)
            t.kf.update_radial_velocity(vr, self.radar_vel_var)

    def update_camera(self, detections, max_bearing_diff=0.05):
        """detections: list of (bearing_rad, label, conf). Assigns class labels to tracks."""
        for bearing, label, conf in detections:
            best, best_diff = None, max_bearing_diff
            for t in self.tracks:
                if t.kf.x[0] <= 0.0:  # behind the camera
                    continue
                diff = abs(np.arctan2(t.kf.x[1], t.kf.x[0]) - bearing)
                if diff < best_diff:
                    best, best_diff = t, diff
            if best is not None:
                best.label = label
                best.confidence = max(best.confidence, conf)

    def confirmed(self, min_hits=3):
        return [t for t in self.tracks if t.hits >= min_hits]
