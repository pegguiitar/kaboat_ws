import math

from kaboat_behaviors.dock_target import target_from_entrance


def test_entrance_xy_becomes_fsm_bearing_and_range():
    target = target_from_entrance(3.0, 4.0, 0.0)
    assert target is not None
    assert math.isclose(target.distance, 5.0)
    assert math.isclose(target.bearing, math.atan2(4.0, 3.0))


def test_invalid_or_behind_entrance_fails_closed():
    for point in ((0, 1, 0), (-1, 0, 0), (1, 0, 0.2),
                  (float('nan'), 0, 0), (20, 0, 0)):
        assert target_from_entrance(*point) is None
