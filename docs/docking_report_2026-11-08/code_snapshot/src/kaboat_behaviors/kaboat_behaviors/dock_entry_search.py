"""Bounded DRI-guided entry search, preserving the station normal and stop point."""
import math
import numpy as np
from .bspline_planner import generate,generate_docking


def find_docking_entry(dri,boat,yaw,velocity,waypoints,heading,params,*,step=.15,
                       radius=1.5,previous_shift=None,bounds=None,margin=.5,approach_only=False):
    if not all(math.isfinite(v) and v>0 for v in (step,radius)):
        raise ValueError('invalid docking entry search limits')
    points=np.asarray(waypoints,float)
    if points.shape!=(3,2) or not np.isfinite(points).all():raise ValueError('invalid docking entry points')
    outward=-np.array([math.cos(heading),math.sin(heading)])
    stop=points[-1].copy();nominal_distance=float((points[0]-stop)@outward)
    alignment_distance=float((points[1]-stop)@outward)
    threshold=params.safe_threshold*params.gen_margin
    info={'safe_path':False,'attempts':0,'shift_m':None,'step_m':step,'radius_m':radius}
    if dri.risk_at(float(boat[0]),float(boat[1]))>params.safe_threshold:
        info['reason']='start lies inside DRI risk region';return points,None,info
    if not approach_only and dri.risk_at(*stop)>threshold:
        info['reason']='stop point lies inside DRI risk region';return points,None,info
    dangerous=np.argwhere(dri.data>threshold)
    nearest=None
    if len(dangerous):
        dangerous_xy=np.c_[(dangerous[:,1]+dri.anchor[0]+.5)*dri.resolution,
                           (dangerous[:,0]+dri.anchor[1]+.5)*dri.resolution]
        nearest=dangerous_xy[np.argmin(np.linalg.norm(dangerous_xy-points[0],axis=1))]
    preferred=1. if nearest is None or (points[0]-nearest)@outward>=0 else -1.
    shifts=[]
    if previous_shift is not None and abs(previous_shift)<=radius+1e-8:shifts.append(float(previous_shift))
    shifts.append(0.)
    for i in range(1,int(math.floor(radius/step))+1):shifts.extend([preferred*i*step,-preferred*i*step])
    tried=set()
    for shift in shifts:
        key=round(shift,8)
        if key in tried:continue
        tried.add(key)
        remaining=nominal_distance+shift
        if remaining<.35:continue
        entry=stop+remaining*outward
        align=stop+min(alignment_distance,.5*remaining)*outward
        candidate=np.array([entry,align,stop]);info['attempts']+=1
        if bounds is not None:
            x0,y0,x1,y1=bounds
            if not np.all((candidate[:,0]>=x0+margin)&(candidate[:,0]<=x1-margin)
                          &(candidate[:,1]>=y0+margin)&(candidate[:,1]<=y1-margin)):continue
        if approach_only:
            distance=float(np.linalg.norm(entry-np.asarray(boat)))
            plan=generate(dri,boat,yaw,velocity,entry,params)
        else:plan=generate_docking(dri,boat,yaw,velocity,candidate,heading,params)
        if plan is None:continue
        if bounds is not None:
            if not np.all((plan.samples[:,0]>=x0+margin)&(plan.samples[:,0]<=x1-margin)
                          &(plan.samples[:,1]>=y0+margin)&(plan.samples[:,1]<=y1-margin)):continue
        info.update(safe_path=True,shift_m=shift,reason='ok',entry_xy=entry.tolist(),
                    nearest_dri_xy=nearest.tolist() if nearest is not None else None)
        return candidate,plan,info
    info['reason']='no safe entry within bounded search'
    return points,None,info
