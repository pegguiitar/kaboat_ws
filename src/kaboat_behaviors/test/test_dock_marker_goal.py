import numpy as np
import pytest
from kaboat_behaviors.dock_marker_goal import marker_goal

POINTS = {'triangle': (-0.2, 2.5), 'circle': (-0.2, 1.), 'square': (-0.2, 4.)}


def test_exact_one_metre_inward_shift_and_class_selection():
    result = marker_goal(POINTS, 'triangle', (5, 2.5), tank_bounds=(0, 0, 10, 5))
    np.testing.assert_allclose(result.goal_xy, (0.8, 2.5))
    np.testing.assert_allclose(result.outward, (1, 0), atol=1e-12)
    assert result.marker_xy == POINTS['triangle']
    assert result.wall_span == 3.0
    assert marker_goal(POINTS, 'circle', (5, 2.5)).goal_xy == pytest.approx((0.8, 1.))


def test_oblique_wall_shift_is_perpendicular_not_current_boat_heading():
    points = {'triangle': (0, 0), 'circle': (1, 1), 'square': (2, 2)}
    result = marker_goal(points, 'triangle', (4, 0))
    np.testing.assert_allclose(result.goal_xy, (2**-.5, -2**-.5))
    assert np.linalg.norm(np.array(result.goal_xy) - result.marker_xy) == pytest.approx(1.0)


def test_bad_geometry_and_outside_goal_are_not_silently_clamped():
    with pytest.raises(ValueError, match='at least two'):
        marker_goal({'triangle': (0, 0)}, 'triangle', (5, 2))
    with pytest.raises(ValueError, match='wall fit'):
        marker_goal({'triangle': (0, 0), 'circle': (0, .1)}, 'triangle', (5, 2))
    with pytest.raises(ValueError, match='wall fit'):
        marker_goal({'triangle': (0, 0), 'circle': (2, 0), 'square': (1, 2)},
                    'triangle', (5, 2), max_rms=.1)
    with pytest.raises(ValueError, match='outside'):
        marker_goal({name: (x-2, y) for name, (x, y) in POINTS.items()},
                    'triangle', (5, 2), tank_bounds=(0, 0, 10, 5))


def test_goal_average_uses_measured_time_window_without_duplicate_weighting():
    from kaboat_behaviors.dock_marker_goal import TimedGoalAverage
    avg = TimedGoalAverage(window_s=1., max_std_m=1.)
    assert not avg.update((1, 2), 10.)[1]['ready']
    assert not avg.update((1.2, 2), 10.2)[1]['ready']
    value, info = avg.update((.8, 2), 10.4)
    np.testing.assert_allclose(value, (1, 2))
    assert info['ready'] and info['samples'] == 3
    avg.update((999, 999), 10.4)  # repeat timer ticks must not add samples
    value, info = avg.update((1, 2), 10.6)
    assert info['samples'] == 4
    np.testing.assert_allclose(value, (1, 2))
    value, info = avg.update((3, 2), 12.)
    assert info['samples'] == 1 and not info['ready']
    np.testing.assert_allclose(value, (3, 2))


def test_unstable_goal_average_is_not_accepted():
    from kaboat_behaviors.dock_marker_goal import TimedGoalAverage
    avg = TimedGoalAverage()
    avg.update((0, 0), 10.)
    avg.update((1, 0), 10.2)
    _, info = avg.update((0, 0), 10.4)
    assert not info['ready'] and info['std_m'] > .25
    assert 'unstable' in info['reason']
