import dataclasses
import math

import pytest

from kaboat_behaviors.docking_fsm import DockParams
from kaboat_behaviors.slot_dimensions import SlotDimensions

OK = dict(wall_to_mouth=2.5, slot_width=3.0, slot_width_tolerance=0.5,
          hull_width=1.0, hull_length=1.0, berth_distance=1.0,
          pre_entry_offset=1.5, exit_clearance=0.2)


def make(**changes):
    return SlotDimensions(**{**OK, **changes})


def test_reverse_distance_matches_the_old_launch_formula():
    # 예전 launch: wall_to_mouth + hull_length/2 + exit_clearance - 1.0
    assert make().reverse_min_distance == pytest.approx(2.5 + 0.5 + 0.2 - 1.0)
    assert make(wall_to_mouth=1.4, berth_distance=0.9,
                exit_clearance=0.0).reverse_min_distance == pytest.approx(1.0)
    assert make(wall_to_mouth=1.3, berth_distance=1.0,
                hull_length=0.4, exit_clearance=0.0).reverse_min_distance == pytest.approx(0.5)


def test_wall_fit_window_follows_slot_width():
    assert make().wall_fit_radius == pytest.approx(1.3)       # 기존 기본값과 동일
    narrow = make(slot_width=2.0, hull_width=0.8).wall_fit_params()
    assert narrow.window_radius == pytest.approx(0.8)
    assert narrow.min_span <= 2 * narrow.window_radius
    wide = make(slot_width=6.0)
    assert wide.wall_fit_radius == 2.0                         # 상한
    assert make(slot_width=1.2, hull_width=0.5).wall_fit_radius == 0.6


def test_window_never_reaches_the_partition_corner():
    for width in (1.2, 2.0, 3.0, 4.5, 8.0):
        dims = make(slot_width=width, hull_width=min(1.0, width / 2))
        assert dims.wall_fit_radius < width / 2 or width < 1.6


def test_fsm_overrides_only_contain_real_dockparams_fields():
    fields = {f.name for f in dataclasses.fields(DockParams)}
    assert set(make().fsm_overrides()) <= fields


def test_defaults_in_dockparams_match_the_shipped_dimensions():
    d, o = DockParams(), make().fsm_overrides()
    for key in ('slot_width', 'slot_width_tolerance', 'hull_width', 'hull_length',
                'berth_distance', 'pre_entry_offset'):
        assert getattr(d, key) == o[key], key


@pytest.mark.parametrize('bad', [
    dict(wall_to_mouth=float('nan')),
    dict(wall_to_mouth=1.2),                  # berth_distance + 0.2 이하
    dict(wall_to_mouth=9.0),                  # 상한 초과
    dict(hull_width=3.0),                     # 선체가 슬롯보다 넓음
    dict(hull_width=0.0),
    dict(berth_distance=0.55),                # 선수가 뒷벽에 닿음(반길이 0.5 + 0.1)
    dict(pre_entry_offset=0.0),
    dict(hull_length=-1.0),
    dict(exit_clearance=-0.1),
    dict(slot_width_tolerance=-0.1),
    dict(slot_width=float('inf')),
])
def test_inconsistent_dimensions_are_rejected(bad):
    with pytest.raises(ValueError):
        make(**bad)
