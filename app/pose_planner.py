"""Bounded Hybrid A*: continuous differential-drive poses, quantized labels.

Search and shortcut costs estimate travel time; neither discretization nor the
finite primitive set promises a globally shortest/fastest continuous route.
"""
from dataclasses import dataclass,fields
import heapq
import itertools
import math
from footprint import FootprintScene,integrate
from pose_provider import angle_difference
from planning_limits import PlanningLimitError,PlanningBudget,PlanningSettings
from grid_map import tracking_margin


@dataclass(frozen=True)
class PoseSettings:
    heading_bins: int = 32
    position_resolution: float = .1
    step: float = .4
    max_states: int = 100000
    reverse_speed: float = .7
    heuristic_weight: float = 1.5

    @classmethod
    def from_config(cls,config):
        values=config.get('pose_planning',{})
        if set(values)-{f.name for f in fields(cls)}:raise ValueError('Unknown pose_planning setting')
        result=cls(**values)
        for field in fields(cls):
            value=getattr(result,field.name)
            if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or value<=0:
                raise ValueError(f'Invalid pose_planning.{field.name}')
        if not isinstance(result.heading_bins,int) or result.heading_bins<8 or result.heading_bins%4:
            raise ValueError('pose_planning.heading_bins must be a multiple of four >= 8')
        if not isinstance(result.max_states,int):raise ValueError('pose_planning.max_states must be an integer')
        if result.heuristic_weight<1:raise ValueError('pose_planning.heuristic_weight must be >= 1')
        return result


@dataclass(frozen=True)
class Motion:
    start: tuple
    v: float
    w: float
    duration: float

    @property
    def end(self):return integrate(self.start,self.v,self.w,self.duration)

    def samples(self):
        n=max(1,math.ceil(max(abs(self.v)*self.duration/.08,abs(self.w)*self.duration/.04)))
        return [integrate(self.start,self.v,self.w,self.duration*i/n) for i in range(n+1)]


class PoseRoute(list):
    """XY compatibility for metrics/CLI, with authoritative pose/motion data."""
    def __init__(self,start,motions,estimated_time):
        self.start=tuple(start);self.motions=tuple(motions);self.estimated_time=estimated_time
        self.poses=[self.start]
        for motion in motions:self.poses.extend(motion.samples()[1:])
        points=[self.start[:2]]
        for motion in motions:
            if math.dist(points[-1],motion.end[:2])>1e-8:points.append(motion.end[:2])
        if len(points)==1:points.append(points[0])
        super().__init__(points)

    def blocks(self):
        """Preserve planned pivots and gear changes; flow through moving arcs."""
        blocks=[]
        for motion in self.motions:
            direction=0 if abs(motion.v)<1e-9 else (1 if motion.v>0 else -1)
            if not blocks or blocks[-1][0]!=direction:
                blocks.append((direction,[motion.start]))
            blocks[-1][1].extend(motion.samples()[1:])
        return blocks


def approach_poses(scene,target,start,contact):
    from contact_goal import target_ray_radius
    config=scene.config;cx,cy=target['center'];front=config['robot']['footprint_bounds'][2]
    extra=tracking_margin(config)
    angles=[i*math.pi/8 for i in range(16)]
    angles.extend((math.atan2(start[1]-cy,start[0]-cx),start[2]+math.pi))
    goals=[];seen=set()
    for angle in angles:
        key=round(angle%(2*math.pi),7)
        if key in seen:continue
        seen.add(key);heading=math.atan2(-math.sin(angle),-math.cos(angle))
        radius=target_ray_radius(target,angle)
        for stand_off in (.3,.65,1.):
            distance=radius+front+scene.margin+extra+stand_off
            pose=(cx+math.cos(angle)*distance,cy+math.sin(angle)*distance,heading)
            if not scene.free(pose,extra):continue
            travel=distance-radius-front+.01
            if contact and not scene.motion_free(pose,1.,0.,max(0.,travel),ignore=target['id']):continue
            goals.append(pose)
    return goals


def distance_guide(config,goals,scene,extra):
    """Obstacle-aware ordering on a relaxed coarse positional map.

    It guides search, never certifies or rejects a pose. Rasterization makes
    this an approximate heuristic; no continuous optimality bound is claimed.
    """
    from grid_map import GridMap
    left,bottom,right,top=config['robot']['footprint_bounds']
    radius=max(0.,min(-left,-bottom,right,top))
    relaxed=dict(config,resolution=max(.2,config['resolution']))
    relaxed['robot']=dict(config['robot'],radius=radius,margin=scene.margin+extra)
    grid=GridMap(relaxed,budget=scene.budget)
    distances={};queue=[]
    for goal in goals:
        cell=grid.cell(goal[:2])
        if grid.free(cell):distances[cell]=0.;heapq.heappush(queue,(0.,cell))
    while queue:
        scene.check_budget()
        distance,cell=heapq.heappop(queue)
        if distance!=distances[cell]:continue
        scene.check_budget(expansion=True)
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nxt=cell[0]+dx,cell[1]+dy
            if not grid.free(nxt):continue
            updated=distance+grid.resolution*math.hypot(dx,dy)
            if updated<distances.get(nxt,math.inf):
                distances[nxt]=updated;heapq.heappush(queue,(updated,nxt))
    return grid,distances


def pose_plan(config,start,target=None,goal=None,contact=False,budget=None,connect_only=False):
    budget=budget or PlanningBudget(PlanningSettings.from_config(config))
    settings=PoseSettings.from_config(config);scene=FootprintScene(config,budget)
    extra=tracking_margin(config)
    if not scene.free(start):raise ValueError('Robot footprint violates current obstacle / operating area clearance')
    # Never silently waive the planning envelope for the first transition.
    goals=approach_poses(scene,target,start,contact) if target is not None else [(*goal,None)]
    if not goals:raise ValueError('No reachable approach/route with safe contact corridor (oriented footprint)')
    x0,y0,x1,y1=config['bounds']
    if budget is not None:
        budget.cells(math.ceil((x1-x0)/config['resolution'])*math.ceil((y1-y0)/config['resolution']))
    vmax=config.get('navigation',{}).get('max_linear_speed',.7)
    wmax=config.get('navigation',{}).get('max_angular_speed',.5)
    reverse=min(vmax,settings.reverse_speed)
    lateral=config.get('path_following',{}).get('lateral_acceleration',.45)
    da=2*math.pi/settings.heading_bins

    def key(pose,direction):
        return (math.floor((pose[0]-x0)/settings.position_resolution),
                math.floor((pose[1]-y0)/settings.position_resolution),
                round((pose[2]%(2*math.pi))/da)%settings.heading_bins,direction)

    guide=None
    def heuristic(pose):
        distance=min(math.dist(pose[:2],g[:2]) for g in goals)
        if guide is not None:
            grid,distances=guide
            distance=max(distance,distances.get(grid.cell(pose[:2]),distance))
        return distance/vmax

    def connector(pose,destination,direction=1):
        distance=math.dist(pose[:2],destination[:2])
        bearing=(math.atan2(destination[1]-pose[1],destination[0]-pose[0])
                 if distance>1e-8 else pose[2])
        heading=bearing if direction>0 else bearing+math.pi
        result=[];current=pose;cost=0.
        for yaw,length in ((heading,distance),(destination[2],0.)):
            if yaw is not None:
                delta=angle_difference(yaw,current[2])
                if abs(delta)>1e-8:
                    motion=Motion(current,0.,math.copysign(wmax,delta),abs(delta)/wmax)
                    if not scene.motion_free(current,motion.v,motion.w,motion.duration,extra):return None
                    result.append(motion);current=motion.end;cost+=motion.duration+.5
            if length>1e-8:
                speed=vmax if direction>0 else reverse
                motion=Motion(current,direction*speed,0.,length/speed)
                if not scene.motion_free(current,motion.v,motion.w,motion.duration,extra):return None
                result.append(motion);current=motion.end;cost+=motion.duration
        if contact and target is not None:
            from contact_goal import target_ray_radius
            from motion_config import ContactSettings
            inward=destination[2]
            gap=max(0.,math.dist(destination[:2],target['center'])-
                    target_ray_radius(target,inward+math.pi)-config['robot']['footprint_bounds'][2])
            cost+=gap/min(vmax,ContactSettings.from_config(config).max_speed)
        return result,cost

    # Fast exact connectors also cover non-bin start and terminal headings.
    best=None
    for destination in sorted(goals,key=lambda g:math.dist(start[:2],g[:2])):
        scene.check_budget()
        for direction in (1,-1):
            tail=connector(start,destination,direction)
            if tail is not None and (best is None or tail[1]<best[1]):best=tail
    if best is not None:
        scene.budget=None
        return scene,PoseRoute(start,*best)

    if connect_only:return None

    guide=distance_guide(config,goals,scene,extra)

    primitives=[]
    for direction in (1,-1):
        speed=vmax if direction>0 else reverse
        for length in (settings.step,2*settings.step):
            for delta in (0.,-da,da):
                curvature=abs(delta)/length
                cap=min(speed,math.sqrt(lateral/max(curvature,1e-9)),wmax/max(curvature,1e-9))
                duration=length/cap
                primitives.append((direction*cap,delta/duration,duration,direction))
    for delta in (-da,da,-4*da,4*da):
        primitives.append((0.,math.copysign(wmax,delta),abs(delta)/wmax,0))
    counter=itertools.count();root=key(start,0)
    queue=[(heuristic(start),0.,next(counter),0)]
    # Immutable records prevent an updated label from changing a child's parent.
    records=[(tuple(start),0,None,())];costs={root:0.}
    closed=set();terminal=None;expansions=0
    while queue:
        scene.check_budget()
        _,cost,_,index=heapq.heappop(queue)
        pose,previous,parent,edge=records[index];label=key(pose,previous)
        if cost!=costs.get(label) or label in closed:continue
        scene.check_budget(expansion=True);closed.add(label);expansions+=1
        nearest=sorted(goals,key=lambda g:math.dist(pose[:2],g[:2]))[:4]
        # Analytic goal connections stop the search from depending on an exact
        # terminal bin; collision certification includes both endpoint pivots.
        if expansions%24==0 or math.dist(pose[:2],nearest[0][:2])<1.5:
            tails=[]
            for destination in nearest:
                for direction in (1,-1):
                    tail=connector(pose,destination,direction)
                    if tail is not None:tails.append(tail)
            if tails:
                terminal=(index,*min(tails,key=lambda t:t[1]));break
        for v,w,duration,direction in primitives:
            nxt=integrate(pose,v,w,duration)
            if not (x0<nxt[0]<x1 and y0<nxt[1]<y1):continue
            next_label=key(nxt,direction)
            if next_label==label or next_label in closed:continue
            updated=cost+duration+(.6 if direction!=previous and previous!=0 else 0.)
            if updated>=costs.get(next_label,math.inf):continue
            if not scene.motion_free(pose,v,w,duration,extra):continue
            if len(records)>=settings.max_states:
                raise PlanningLimitError(f'Pose planning state limit reached ({settings.max_states} states)')
            costs[next_label]=updated
            records.append((nxt,direction,index,(Motion(pose,v,w,duration),)))
            heapq.heappush(queue,(updated+settings.heuristic_weight*heuristic(nxt),updated,next(counter),len(records)-1))
    if terminal is None:
        raise ValueError('No reachable approach/route with oriented footprint at configured search resolution')
    index,tail,tail_cost=terminal;motions=[];estimate=costs[key(records[index][0],records[index][1])]+tail_cost
    while index is not None:
        pose,previous,parent,edge=records[index]
        motions.extend(reversed(edge));index=parent
    motions=list(reversed(motions))+tail
    scene.check_budget();scene.budget=None
    print(f'POSE_PLAN: states={len(records)}, expansions={expansions}, estimated_time={estimate:.2f}s',flush=True)
    return scene,PoseRoute(start,motions,estimate)
