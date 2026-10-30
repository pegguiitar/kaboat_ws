import math

from kaboat_behaviors.dock_target import (
    plan_from_points, target_from_entrance, target_from_point)


def test_entrance_xy_becomes_fsm_bearing_and_range():
    target = target_from_entrance(3.0, 4.0, 0.0)
    assert target is not None
    assert math.isclose(target.distance, 5.0)
    assert math.isclose(target.bearing, math.atan2(4.0, 3.0))


def test_invalid_or_behind_entrance_fails_closed():
    for point in ((0, 1, 0), (-1, 0, 0), (1, 0, 0.2),
                  (float('nan'), 0, 0), (20, 0, 0)):
        assert target_from_entrance(*point) is None


def test_marker_height_is_ignored_for_horizontal_berth_distance():
    target = target_from_point(0.8, 0.6, 1.2, require_ground=False)
    assert target is not None
    assert math.isclose(target.distance, 1.0)


def test_three_waypoints_are_collinear_and_in_order():
    # Boat at x=0; wall marker x=6, mouth x=3, outside waypoint x=1.5.
    plan = plan_from_points(6.0, 0.0, 3.0, 0.0,
                            pre_entry_offset=1.5, berth_distance=1.0)
    assert plan is not None
    assert math.isclose(plan.pre_entry.distance, 1.5)
    assert math.isclose(plan.mouth.distance, 3.0)
    assert math.isclose(plan.berth.distance, 5.0)
    assert math.isclose(plan.marker.distance, 6.0)
    assert math.isclose(plan.axis_error, 0.0)


def test_dock_plan_rejects_reversed_or_too_shallow_geometry():
    assert plan_from_points(3.0, 0.0, 6.0, 0.0,
                            pre_entry_offset=1.5, berth_distance=1.0) is None
    assert plan_from_points(3.0, 0.0, 2.0, 0.0,
                            pre_entry_offset=1.5, berth_distance=1.0) is None


def test_waypoint_at_current_boat_position_is_valid():
    # W1 = 0; reaching it must advance the FSM instead of losing the plan.
    plan = plan_from_points(4.5, 0.0, 1.5, 0.0,
                            pre_entry_offset=1.5, berth_distance=1.0)
    assert plan is not None
    assert plan.pre_entry.distance == 0.0
