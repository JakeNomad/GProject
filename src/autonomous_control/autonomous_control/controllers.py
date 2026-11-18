"""Pure Pursuit (lateral) and PID (longitudinal) controllers. No ROS dependency."""
import math


class PID:

    def __init__(self, kp, ki, kd, i_limit=1.0):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def step(self, error, dt):
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral + error * dt))
        deriv = 0.0 if self.prev_error is None or dt <= 0 else (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * deriv


def pure_pursuit_steer(ego_x, ego_y, ego_yaw, path_xy, speed, wheelbase,
                       lookahead_gain=0.8, min_lookahead=4.0):
    """Returns the front-wheel steering angle [rad] (positive = left) or None if
    the path is exhausted."""
    lookahead = max(min_lookahead, lookahead_gain * speed)
    # closest point, then walk forward until the look-ahead distance is reached
    closest = min(range(len(path_xy)),
                  key=lambda i: (path_xy[i][0] - ego_x) ** 2 + (path_xy[i][1] - ego_y) ** 2)
    target = None
    for px, py in path_xy[closest:]:
        if math.hypot(px - ego_x, py - ego_y) >= lookahead:
            target = (px, py)
            break
    if target is None:
        if closest == len(path_xy) - 1:
            return None
        target = path_xy[-1]
    alpha = math.atan2(target[1] - ego_y, target[0] - ego_x) - ego_yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    return math.atan2(2.0 * wheelbase * math.sin(alpha), lookahead)
