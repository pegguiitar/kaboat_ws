"""Associate marker sight rays with scan lines; no camera depth or ROS dependency."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class RayWallParams:
    angular_window: float = math.radians(15)
    inlier_distance: float = .06
    min_points: int = 12
    min_span: float = .65
    max_rms: float = .045
    min_fraction: float = .45
    iterations: int = 100
    max_range: float = 12.
    direct_extrapolation: float = .35
    shared_extrapolation: float = 2.
    min_incidence: float = .3
    max_shared_angle: float = math.radians(12)
    max_shared_offset: float = .25
    hint_angle: float = math.radians(25)
    hint_min_fraction: float = .2


@dataclass(frozen=True)
class RayWall:
    center: tuple
    tangent: tuple
    normal: tuple       # towards the observing camera / water
    low: float
    high: float
    rms: float
    points: int

    @property
    def span(self):
        return self.high - self.low


@dataclass(frozen=True)
class WallObservation:
    point: tuple        # camera sight ray / LiDAR line intersection
    wall: RayWall
    source: str         # class that directly supplied the fitted line
    shared: bool


def _ray(origin, direction):
    o, d = np.asarray(origin, float), np.asarray(direction, float)
    if o.shape != (2,) or d.shape != (2,) or not np.isfinite([o, d]).all() or np.linalg.norm(d) < 1e-6:
        raise ValueError('invalid marker ray')
    return o, d / np.linalg.norm(d)


def intersect_ray(wall, origin, direction, params, *, shared=False):
    o, d = _ray(origin, direction)
    n, c, tangent = map(np.asarray, (wall.normal, wall.center, wall.tangent))
    denominator = float(n @ d)
    if denominator > -params.min_incidence:
        return None
    distance = float(n @ (c-o) / denominator)
    if not .25 <= distance <= params.max_range:
        return None
    hit = o + distance*d
    along = float((hit-c) @ tangent)
    extra = params.shared_extrapolation if shared else params.direct_extrapolation
    if not wall.low-extra <= along <= wall.high+extra:
        return None
    return tuple(hit)


def fit_ray_wall(points, origin, direction, params=None, *, normal_hint=None):
    p = params or RayWallParams()
    o, d = _ray(origin, direction)
    hint=None if normal_hint is None else _ray((0,0),normal_hint)[1]
    required_fraction=p.min_fraction if hint is None else min(p.min_fraction,p.hint_min_fraction)
    xy = np.asarray(points, float).reshape(-1, 2)
    xy = xy[np.isfinite(xy).all(axis=1)]
    rel = xy-o
    ranges = np.linalg.norm(rel, axis=1)
    angle = np.arctan2(rel[:, 0]*d[1]-rel[:, 1]*d[0], rel @ d)
    candidates = xy[(ranges>.25)&(ranges<p.max_range)&(abs(angle)<=p.angular_window)]
    if len(candidates)<p.min_points:
        return None
    rng = np.random.default_rng(0)
    pairs=rng.integers(0,len(candidates),size=(p.iterations,2))
    starts=candidates[pairs[:,0]]
    deltas=candidates[pairs[:,1]]-starts
    lengths=np.linalg.norm(deltas,axis=1)
    usable=lengths>=p.min_span*.5
    starts,deltas,lengths=starts[usable],deltas[usable],lengths[usable]
    if not len(lengths):return None
    normals=np.c_[-deltas[:,1],deltas[:,0]]/lengths[:,None]
    if hint is not None:
        signs=np.where(np.sum(normals*(o-starts),axis=1)>=0,1.,-1.)
        normals*=signs[:,None]
        aligned=normals@hint>=math.cos(p.hint_angle)
        starts,normals=starts[aligned],normals[aligned]
        if not len(normals):return None
    # Evaluate all hypotheses together; avoid hundreds of tiny Python/NumPy calls.
    distances=(normals[:,0,None]*(candidates[None,:,0]-starts[:,0,None])
               +normals[:,1,None]*(candidates[None,:,1]-starts[:,1,None]))
    masks=abs(distances)<=p.inlier_distance
    counts=masks.sum(axis=1)
    order=np.argsort(-counts,kind='stable')
    best,best_score=None,None
    seen=set()
    for index in order:
        count=int(counts[index])
        if count<max(p.min_points,required_fraction*len(candidates)):
            break
        if best_score is not None and count<=best_score[0]:
            break
        keep=masks[index]
        key=np.packbits(keep).tobytes()
        if key in seen:continue
        seen.add(key)
        inliers = candidates[keep]
        for _ in range(2):
            center = inliers.mean(axis=0)
            _, _, vh = np.linalg.svd(inliers-center, full_matrices=False)
            tangent, normal = vh[0], vh[1]
            keep = abs((candidates-center)@normal)<=p.inlier_distance
            inliers = candidates[keep]
        if len(inliers)<max(p.min_points,required_fraction*len(candidates)):
            continue
        center = inliers.mean(axis=0)
        _, _, vh = np.linalg.svd(inliers-center, full_matrices=False)
        tangent, normal = vh[0], vh[1]
        if normal@(o-center)<0:
            normal=-normal
        if hint is not None and normal@hint<math.cos(p.hint_angle):
            continue
        along=(inliers-center)@tangent
        rms=float(np.sqrt(np.mean(((inliers-center)@normal)**2)))
        wall=RayWall(tuple(center),tuple(tangent),tuple(normal),float(along.min()),float(along.max()),rms,len(inliers))
        if wall.span<p.min_span or rms>p.max_rms:
            continue
        hit=intersect_ray(wall,o,d,p)
        if hit is None:
            continue
        # Prefer support; distance breaks ties towards the nearest visible line.
        score=(len(inliers),-float(np.linalg.norm(np.asarray(hit)-o)))
        if best_score is None or score>best_score:
            best,best_score=wall,score
    return best


def compatible_walls(a, b, p):
    return (np.dot(a.normal,b.normal)>=math.cos(p.max_shared_angle)
            and abs(np.dot(np.asarray(b.center)-a.center,a.normal))<=p.max_shared_offset
            and abs(np.dot(np.asarray(b.center)-a.center,b.normal))<=p.max_shared_offset)


def select_station_wall(direct, params, *, normal_hint=None, previous_wall=None):
    """Select a coherent station plane; never average perpendicular wall families."""
    candidates={c:w for c,w in direct.items() if w is not None}
    if normal_hint is not None:
        hint=_ray((0,0),normal_hint)[1]
        candidates={c:w for c,w in candidates.items() if np.dot(w.normal,hint)>=math.cos(params.hint_angle)}
    if previous_wall is not None:
        candidates={c:w for c,w in candidates.items() if compatible_walls(w,previous_wall,params)}
    groups=[]
    for c,w in candidates.items():
        group=tuple(sorted(k for k,v in candidates.items() if compatible_walls(w,v,params)))
        # Pairwise compatibility prevents a chain of unrelated planes merging.
        if group and all(compatible_walls(candidates[x],candidates[y],params) for x in group for y in group):
            if group not in groups:groups.append(group)
    if not groups:return None,(),'no station plane satisfies orientation/history'
    votes=max(map(len,groups));best=[g for g in groups if len(g)==votes]
    if len(best)!=1:return None,(),'ambiguous station wall families'
    group=best[0]
    # Length ranks only candidates already confirmed to be the same plane.
    source=max(group,key=lambda c:(candidates[c].span,candidates[c].points,-candidates[c].rms))
    return (source,candidates[source]),group,'ok'


def associate_walls(points, rays, params=None, *, normal_hint=None, previous_wall=None, selection=None):
    """Select one current station plane and share its normal across visible rays.

    A configured station orientation can recover a smaller back-wall patch even
    when a perpendicular side wall has more scan support. Temporal history is
    only a gate on CURRENT scan fits, never a source of synthetic fresh points.
    """
    p=params or RayWallParams()
    hint=normal_hint if normal_hint is not None else (previous_wall.normal if previous_wall else None)
    direct={name:fit_ray_wall(points,*ray,p,normal_hint=hint) for name,ray in rays.items()}
    chosen,support,reason=select_station_wall(direct,p,normal_hint=hint,previous_wall=previous_wall)
    if selection is not None:
        selection.update(reason=reason,support_classes=list(support),orientation_prior=hint is not None)
    if chosen is None:return {}
    source,wall=chosen
    if selection is not None:
        selection['source']=source
        selection['selection_basis']='orientation_history_consensus'
    observations={}
    for name,ray in rays.items():
        shared=name!=source
        hit=intersect_ray(wall,*ray,p,shared=shared)
        if hit is not None:observations[name]=WallObservation(hit,wall,source,shared)
    return observations
