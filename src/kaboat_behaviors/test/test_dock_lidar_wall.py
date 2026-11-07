import math
import numpy as np
import pytest
from kaboat_behaviors.dock_lidar_wall import RayWallParams, fit_ray_wall, associate_walls


def wall(x=5.,low=-3.,high=3.,n=300):
    y=np.linspace(low,high,n)
    return np.c_[np.full(n,x),y]


def test_single_marker_fits_lidar_wall_without_depth():
    points=wall();ray=((0,0),(1,.1))
    result=associate_walls(points,{'square':ray})['square']
    assert result.point==pytest.approx((5,.5))
    assert result.wall.normal==pytest.approx((-1,0))
    assert result.source=='square' and not result.shared


def test_camera_origin_offset_and_scan_outliers():
    rng=np.random.default_rng(8)
    points=np.r_[wall(),rng.uniform([.5,-4],[8,4],size=(60,2))]
    result=associate_walls(points,{'triangle':((-.73,0),(1,.1))})['triangle']
    assert result.point==pytest.approx((5,.573),abs=.015)
    assert result.wall.rms<.02


def test_occluded_target_shares_neighbor_line_with_bounded_extension():
    points=wall(low=-.7,high=.7,n=100)
    rays={'circle':((0,0),(1,0)), 'square':((0,0),(1,.4))}
    result=associate_walls(points,rays)
    assert result['square'].shared
    assert result['square'].source=='circle'
    assert result['square'].point==pytest.approx((5,2))
    assert result['square'].wall.normal==pytest.approx((-1,0))


def test_no_line_no_output_and_no_unbounded_sharing():
    assert associate_walls([],{'triangle':((0,0),(1,0))})=={}
    result=associate_walls(wall(low=-.7,high=.7,n=100),{
        'circle':((0,0),(1,0)),'square':((0,0),(1,1))})
    assert 'circle' in result and 'square' not in result


def test_short_foreground_patch_is_not_a_station_wall():
    assert fit_ray_wall(wall(x=2,low=-.15,high=.15), (0,0),(1,0)) is None


def test_rotated_wall_normal_faces_boat():
    angle=.4;rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    points=wall()@rotation.T;ray=np.array([1,0])@rotation.T
    result=associate_walls(points,{'triangle':((0,0),ray)})['triangle']
    assert result.point==pytest.approx(np.array([5,0])@rotation.T)
    assert result.wall.normal==pytest.approx(np.array([-1,0])@rotation.T)


def test_duplicate_points_and_parallel_grazing_ray_are_rejected():
    assert fit_ray_wall(np.repeat([[4,0]],30,axis=0),(0,0),(1,0)) is None
    assert fit_ray_wall(wall(),(0,0),(0,1)) is None


def test_shared_normal_is_current_measurement_only():
    rays={'circle':((0,0),(1,0)),'square':((0,0),(1,.4))}
    assert associate_walls(wall(low=-.7,high=.7,n=100),rays)['square'].shared
    assert associate_walls([],rays)=={}


def test_perpendicular_wall_tie_is_ambiguous_without_station_prior():
    from kaboat_behaviors.dock_lidar_wall import RayWall, select_station_wall
    p=RayWallParams()
    back=RayWall((5,0),(0,1),(-1,0),-2,2,.01,80)
    side=RayWall((0,4),(1,0),(0,-1),-4,4,.01,200)
    chosen,_,reason=select_station_wall({'triangle':back,'square':side},p)
    assert chosen is None and 'ambiguous' in reason
    chosen,support,_=select_station_wall({'triangle':back,'square':side},p,normal_hint=(-1,0))
    assert chosen[0]=='triangle' and support==('triangle',)


def test_majority_support_beats_a_long_perpendicular_side_wall():
    from kaboat_behaviors.dock_lidar_wall import RayWall, select_station_wall
    back=RayWall((5,0),(0,1),(-1,0),-1,1,.01,40)
    side=RayWall((0,4),(1,0),(0,-1),-5,5,.01,300)
    chosen,support,_=select_station_wall({'triangle':back,'circle':back,'square':side},RayWallParams())
    assert chosen[1].normal==(-1,0) and len(support)==2


def test_temporal_prior_blocks_side_wall_switch_and_does_not_supply_missing_scan():
    from kaboat_behaviors.dock_lidar_wall import RayWall, select_station_wall
    back=RayWall((5,0),(0,1),(-1,0),-2,2,.01,80)
    side=RayWall((0,4),(1,0),(0,-1),-4,4,.01,200)
    chosen,_,_=select_station_wall({'square':side},RayWallParams(),previous_wall=back)
    assert chosen is None
    assert associate_walls([],{'square':((0,0),(1,.1))},previous_wall=back)=={}


def test_back_wall_can_be_found_when_perpendicular_side_wall_has_more_points():
    # Angular ROI contains a dense side-wall fragment and a smaller true back wall.
    back=wall(x=5,low=.7,high=1.7,n=50)
    x=np.linspace(2.5,5.,180);side=np.c_[x,np.full(len(x),1.8)]
    result=associate_walls(np.r_[back,side],{'triangle':((0,0),(1,.24))},normal_hint=(-1,0))
    assert result['triangle'].wall.normal==pytest.approx((-1,0),abs=.02)
    assert result['triangle'].point[0]==pytest.approx(5,abs=.03)
