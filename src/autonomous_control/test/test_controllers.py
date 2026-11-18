import math

from autonomous_control.controllers import PID, pure_pursuit_steer


def test_pure_pursuit_straight_path_gives_zero_steer():
    path = [(float(x), 0.0) for x in range(50)]
    assert abs(pure_pursuit_steer(0.0, 0.0, 0.0, path, 5.0, 2.8)) < 1e-6


def test_pure_pursuit_steers_left_for_left_target():
    path = [(float(x), 0.5 * x) for x in range(50)]
    assert pure_pursuit_steer(0.0, 0.0, 0.0, path, 5.0, 2.8) > 0.0


def test_pure_pursuit_returns_none_at_path_end():
    path = [(0.0, 0.0), (1.0, 0.0)]
    assert pure_pursuit_steer(1.0, 0.0, 0.0, path, 0.0, 2.8) is None


def test_pid_sign_and_integral_clamp():
    pid = PID(1.0, 1.0, 0.0, i_limit=0.5)
    for _ in range(100):
        u = pid.step(1.0, 0.1)
    assert math.isclose(u, 1.5)
