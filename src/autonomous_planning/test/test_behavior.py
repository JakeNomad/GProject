"""Tests for the pure decision function. Requires the built autonomous_interfaces
package (run via `colcon test`)."""
import pytest

bp = pytest.importorskip('autonomous_planning.behavior_planner_node')
BehaviorState = bp.BehaviorState


def test_empty_road_is_lane_follow():
    state, speed, lead = bp.decide([], 5.0, 8.0)
    assert state == BehaviorState.LANE_FOLLOW and speed == 8.0 and lead == 0


def test_fast_approach_triggers_emergency_brake():
    state, _, lead = bp.decide([(7, 5.0, 0.0, -10.0, 'car')], 10.0, 8.0)
    assert state == BehaviorState.EMERGENCY_BRAKE and lead == 7


def test_pedestrian_ahead_stops():
    state, speed, _ = bp.decide([(3, 8.0, 0.5, 0.0, 'pedestrian')], 2.0, 8.0)
    assert state == BehaviorState.STOP and speed == 0.0


def test_object_outside_corridor_ignored():
    state, _, _ = bp.decide([(3, 8.0, 5.0, -10.0, 'car')], 5.0, 8.0)
    assert state == BehaviorState.LANE_FOLLOW
