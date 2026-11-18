import numpy as np

from autonomous_fusion.kalman_filter import ConstantVelocityKF, MultiObjectTracker


def test_kf_converges_to_constant_velocity():
    kf = ConstantVelocityKF(0.0, 0.0)
    dt = 0.1
    rng = np.random.default_rng(0)
    for k in range(1, 100):
        kf.predict(dt)
        kf.update_position(2.0 * k * dt + rng.normal(0, 0.1), 1.0 + rng.normal(0, 0.1), 0.01)
    assert abs(kf.x[2] - 2.0) < 0.3
    assert abs(kf.x[3]) < 0.3
    assert abs(kf.x[1] - 1.0) < 0.2


def test_radial_velocity_update_moves_velocity_estimate():
    kf = ConstantVelocityKF(10.0, 0.0)
    for _ in range(20):
        kf.update_radial_velocity(-5.0, 0.1)
    assert abs(kf.x[2] + 5.0) < 0.5


def test_tracker_keeps_identity_and_drops_lost_tracks():
    trk = MultiObjectTracker(max_misses=2)
    for k in range(5):
        trk.predict(0.1)
        trk.update_lidar([(10.0 + 0.1 * k, 0.0, (4, 2, 1.5)), (20.0, 5.0, (4, 2, 1.5))])
    assert len(trk.tracks) == 2
    ids = {t.id for t in trk.tracks}
    assert len(trk.confirmed(3)) == 2

    for _ in range(3):
        trk.predict(0.1)
        trk.update_lidar([(10.6, 0.0, (4, 2, 1.5))])
    assert len(trk.tracks) == 1
    assert trk.tracks[0].id in ids


def test_camera_assigns_label_by_bearing():
    trk = MultiObjectTracker()
    trk.update_lidar([(10.0, 0.0, (4, 2, 1.5)), (10.0, 10.0, (1, 1, 1.8))])
    trk.update_camera([(np.arctan2(10.0, 10.0), 'pedestrian', 0.9)])
    labels = {round(t.kf.x[1]): t.label for t in trk.tracks}
    assert labels[10] == 'pedestrian'
    assert labels[0] == 'unknown'
