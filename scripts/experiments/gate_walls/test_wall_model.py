"""Tests for the experimental geometry and conservative acquisition rules."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from wall_model import WallModel, directional_distance


def seed():
    return SimpleNamespace(left=(3., 1.5), right=(3., -1.5), midpoint=(3., 0.), direction=(1., 0.))


def test_longitudinal_discount_rotates_with_reference_direction():
    assert directional_distance([3., 0.], 0.) == pytest.approx(1.)
    assert directional_distance([0., 3.], 0.) == pytest.approx(3.)
    assert directional_distance([0., 3.], math.pi/2) == pytest.approx(1.)


def test_unverified_objects_do_not_create_walls():
    model = WallModel()
    posts = [(x, y) for x in (3., 6., 9.) for y in (-1.5, 1.5)]
    for k in range(8):
        result = model.update(posts, (0., 0.), 0., (12., 0.), k*.1)
    assert result['status'] == 'WAIT_SEED'


def test_two_parallel_banks_remain_separate_when_boat_steers():
    model = WallModel('adaptive')
    posts = [(x, y) for x in (3., 6., 9.) for y in (-1.5, 1.5)]
    for k in range(20):
        result = model.update(posts, (0., 0.), .6 if k > 8 else 0., (12., 0.), k*.1, seed())
    assert result['status'] == 'TRACK'
    assert result['counts'] == [3, 3]
    assert result['width_min'] == pytest.approx(3.)
    assert result['angle'] == pytest.approx(0., abs=.01)
    labels = np.array(result['labels'])
    xy = np.array(result['xy'])
    assert np.all(xy[labels == 0, 1] > 0)
    assert np.all(xy[labels == 1, 1] < 0)


def test_one_pair_only_is_a_hypothesis_without_supported_direction():
    model = WallModel()
    for k in range(10):
        result = model.update([(3., 1.5), (3., -1.5)], (0., 0.), 0., (12., 0.), k*.1, seed())
    assert result['status'] == 'WARMUP_OR_UNSUPPORTED'


def test_new_observations_update_bend_without_reassigning_old_banks():
    model = WallModel()
    initial = [(3., 1.5), (3., -1.5), (6., 1.5), (6., -1.5)]
    for k in range(10):
        before = model.update(initial, (0., 0.), 0., (14., -1.), k*.1, seed())
    center_before = np.array(before['center'])[-1, 1]
    later = initial + [(9., .9), (9., -2.1)]
    for k in range(10, 30):
        after = model.update(later, (4., 0.), -.2, (14., -1.), k*.1, seed())
    assert after['status'] == 'TRACK'
    assert np.array(after['center'])[-1, 1] < center_before-.2
    labels = np.array(after['labels'])
    xy = np.array(after['xy'])
    assert np.all(labels[np.isclose(xy[:, 0], 3.) & (xy[:, 1] > 0)] == 0)


def test_lost_one_bank_stops_supported_wall_output():
    model = WallModel()
    posts = [(x, y) for x in (3., 6.) for y in (-1.5, 1.5)]
    for k in range(10):
        result = model.update(posts, (0., 0.), 0., (12., 0.), k*.1, seed())
    assert result['status'] == 'TRACK'
    result = model.update([(3., 1.5), (6., 1.5)], (0., 0.), 0., (12., 0.), 12., seed())
    assert result['status'] != 'TRACK'


def test_new_seed_post_keeps_identity_until_memory_confirmation():
    model = WallModel()
    # Two older posts make the memory ready, while the right seed just arrived.
    for k in range(3):
        model.update([(3., 1.5), (6., 1.5)], (0., 0.), 0., (12., 0.), k*.1)
    posts = [(x, y) for x in (3., 6.) for y in (-1.5, 1.5)]
    for k in range(3, 12):
        result = model.update(posts, (0., 0.), 0., (12., 0.), k*.1, seed())
    assert result['status'] == 'TRACK'
    assert result['counts'] == [2, 2]


def test_initial_position_error_is_corrected_by_later_observations():
    model = WallModel()
    shifted = [(x, y+.35) for x in (3., 6.) for y in (-1.5, 1.5)]
    bad_seed = SimpleNamespace(left=(3., 1.85), right=(3., -1.15),
                               midpoint=(3., .35), direction=(1., 0.))
    for k in range(6):
        result = model.update(shifted, (0., 0.), 0., (12., 0.), k*.1, bad_seed)
    before = abs(np.array(result['center'])[:, 1].mean())
    correct = [(x, y) for x in (3., 6., 9.) for y in (-1.5, 1.5)]
    for k in range(6, 40):
        result = model.update(correct, (0., 0.), 0., (12., 0.), k*.1)
    assert abs(np.array(result['center'])[:, 1].mean()) < before/3


def test_staggered_channel_can_make_diagonal_closer_than_same_bank():
    # A directional reward alone cannot identify the two banks.
    own_next = directional_distance([3.5, 1.3], 0.)
    opposite_next = directional_distance([3.5, 1.3-2.4], 0.)
    assert opposite_next < own_next
