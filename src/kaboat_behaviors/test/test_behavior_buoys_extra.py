"""behavior_base 부표맵 기하 — 실패 시 안전(C) 보강: odom 이 없으면 보이는 부표도 없다."""
from types import SimpleNamespace

import pytest

from kaboat_behaviors.behavior_base import BehaviorBase


@pytest.mark.check('C')
def test_visible_buoys_is_empty_without_odom():
    me = SimpleNamespace(odom=None, buoys=[
        SimpleNamespace(color='red', position=SimpleNamespace(x=3.0, y=0.0))])
    me.buoy_range_bearing = lambda b: BehaviorBase.buoy_range_bearing(me, b)
    assert BehaviorBase.visible_buoys(me) == []
    assert BehaviorBase.visible_buoys(me, color='red') == []
