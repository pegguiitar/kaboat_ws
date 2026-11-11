import math
import numpy as np
import pytest
from scipy.interpolate import BSpline
from kaboat_behaviors.bspline_planner import PlannerParams, generate_docking, check, repair, _KNOTS
from kaboat_behaviors.dri import build_dri, DriParams
from test_dri import make_grid

WAYPOINTS=np.array([[3.,1.],[4.,1.],[5.,1.]])


def scene(obstacles=(),boat=(0.,0.)):
    grid,_=make_grid(obstacles,boat_xy=boat)
    return build_dri(grid,boat,0.,DriParams())


def test_bspline_reaches_approach_with_terminal_heading_then_exact_alignment_and_stop():
    plan=generate_docking(scene(),(0,0),0.,(0,0),WAYPOINTS,0.,PlannerParams())
    assert plan is not None and plan.docking_phase=='APPROACH'
    np.testing.assert_allclose(plan.cps[-1],WAYPOINTS[0])
    tangent=BSpline(_KNOTS,plan.cps,3).derivative(1)(1.)
    assert math.atan2(tangent[1],tangent[0])==pytest.approx(0.)
    np.testing.assert_allclose(plan.samples[-1],WAYPOINTS[-1])
    assert np.min(np.linalg.norm(plan.samples-WAYPOINTS[1],axis=1))<1e-8
    assert check(plan,scene(),(0,0),PlannerParams()) is None


def test_turning_start_heading_is_preserved_and_endpoint_faces_wall():
    plan=generate_docking(scene(),(0,0),math.pi/2,(0,0),WAYPOINTS,0.,PlannerParams())
    assert plan is not None
    np.testing.assert_allclose(plan.cps[1]-plan.cps[0],[0,np.linalg.norm(plan.cps[1]-plan.cps[0])],atol=1e-8)
    assert plan.cps[-1,1]==pytest.approx(plan.cps[-2,1])


def test_obstacle_on_final_corridor_blocks_entire_docking_path():
    assert generate_docking(scene([(4.3,1.)]),(0,0),0.,(0,0),WAYPOINTS,0.,PlannerParams()) is None


def test_final_phase_never_routes_back_to_approach_point():
    boat=(4.2,1.)
    plan=generate_docking(scene(boat=boat),boat,0.,(0,0),WAYPOINTS,0.,PlannerParams())
    assert plan is not None and plan.docking_phase=='FINAL'
    assert min(plan.samples[:,0])>=boat[0]-1e-8
    np.testing.assert_allclose(plan.samples[-1],WAYPOINTS[-1])
    assert not repair(plan,scene([(4.6,1.)],boat=boat),0,PlannerParams())


def test_repairs_preserve_terminal_tangent_and_straight_tail():
    plan=generate_docking(scene(),(0,0),0.,(0,0),WAYPOINTS,0.,PlannerParams())
    endpoint=plan.cps[-2:].copy();tail=plan.tail_samples.copy()
    repair(plan,scene([(1.6,.4)]),0,PlannerParams())
    np.testing.assert_allclose(plan.cps[-2:],endpoint)
    np.testing.assert_allclose(plan.samples[-len(tail):],tail)


def test_noncollinear_docking_waypoints_are_rejected():
    with pytest.raises(ValueError):
        generate_docking(scene(),(0,0),0.,(0,0),[[3,1],[4,2],[5,1]],0.,PlannerParams())


@pytest.mark.parametrize('initial_yaw',[0.,math.pi/2])
def test_closed_loop_kinematic_boat_reaches_stop_after_approach(initial_yaw):
    """Controller scenario test; idealized actuator model, not a hardware trial."""
    from kaboat_behaviors.avoid_fsm import AvoidFsm,FsmParams
    from kaboat_behaviors.dri import DriGrid
    p=PlannerParams(finite_goal=True);f=FsmParams()
    dri=DriGrid(np.zeros((120,120),np.float32),(-30,-30),.2,np.empty((0,2)))
    callback=lambda d,b,y,v,g,p,t:generate_docking(d,b,y,v,WAYPOINTS,0.,p,t)
    fsm=AvoidFsm(p,f,1.4,.16,generator=callback)
    xy=np.array([0.,0.]);yaw=initial_yaw;rate=0.;speed=0.
    for tick in range(1200):
        velocity=speed*np.array([math.cos(yaw),math.sin(yaw)])
        out=fsm.step(dri,xy,yaw,velocity,rate,WAYPOINTS[-1],tick*.05)
        assert out.state=='FOLLOW'
        cmd=max(0.,min(.1,out.linear_x))
        speed=f.v_full*(cmd/f.cmd_full)**(1/f.cmd_exp)
        rate=float(np.clip(out.angular_z,-.31,.31))*.52/.31
        yaw+=rate*.05;xy+=speed*np.array([math.cos(yaw),math.sin(yaw)])*.05
        angle=abs(math.atan2(math.sin(yaw),math.cos(yaw)))
        if np.linalg.norm(xy-WAYPOINTS[-1])<.25 and angle<math.radians(15):break
    assert np.linalg.norm(xy-WAYPOINTS[-1])<.25
    assert angle<math.radians(15)
