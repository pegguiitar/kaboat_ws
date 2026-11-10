import numpy as np
from kaboat_behaviors.dock_entry_search import find_docking_entry
from kaboat_behaviors.bspline_planner import PlannerParams,generate_docking,check
from kaboat_behaviors.dri import build_dri,DriParams
from test_dri import make_grid

P=PlannerParams(finite_goal=True)
POINTS=np.array([[3.,0.],[4.,0.],[5.,0.]])


def scene(obstacles):
    grid,_=make_grid(obstacles)
    return build_dri(grid,(0,0),0.,DriParams())


def test_entry_moves_in_small_steps_until_collision_checked_path_exists():
    dri=scene([(2.5,.4)])
    assert generate_docking(dri,(0,0),0.,(0,0),POINTS,0.,P) is None
    points,plan,info=find_docking_entry(dri,(0,0),0.,(0,0),POINTS,0.,P)
    assert plan is not None and info['safe_path'] and info['attempts']>1
    assert abs(info['shift_m'])<=1.5 and abs(info['shift_m']/.15-round(info['shift_m']/.15))<1e-8
    np.testing.assert_allclose(points[-1],POINTS[-1])
    assert check(plan,dri,(0,0),P) is None


def test_unsafe_start_or_stop_is_not_fixed_by_ignoring_risk():
    for obstacle in [(0.,0.),(5.,0.)]:
        _,plan,info=find_docking_entry(scene([obstacle]),(0,0),0.,(0,0),POINTS,0.,P)
        assert plan is None and not info['safe_path'] and info['attempts']==0


def test_valid_previous_shift_is_kept_without_jittering_back_to_nominal():
    _,plan,info=find_docking_entry(scene([]),(0,0),0.,(0,0),POINTS,0.,P,previous_shift=.3)
    assert plan is not None and info['shift_m']==.3 and info['attempts']==1


def test_approach_only_can_run_when_future_final_dock_point_is_occupied():
    dri=scene([(5.,0.)])
    points,plan,info=find_docking_entry(dri,(0,0),0.,(0,0),POINTS,0.,P,approach_only=True)
    assert plan is not None and info['safe_path']
    np.testing.assert_allclose(plan.samples[-1],points[0])
    assert check(plan,dri,(0,0),P) is None
