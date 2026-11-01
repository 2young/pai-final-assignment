"""Bounded expanding search windows inside a fixed robot operating area."""
import math
from grid_map import GridMap
from planning_limits import PlanningBudget,PlanningSettings,PlanningLimitError,PlanningSlice
from path_planner import plan
from target_resolver import plan_to_object
from contact_goal import plan_contact


def planning_windows(config,start,goal,settings):
    operating=config['bounds'];resolution=config['resolution'];seen=set()
    for attempt in range(settings.max_attempts):
        if attempt==settings.max_attempts-1 and settings.max_attempts>1:
            window=list(operating)
        else:
            padding=settings.initial_padding+attempt*settings.expansion_step
            window=[max(operating[0],math.floor((min(start[0],goal[0])-padding)/resolution)*resolution),
                    max(operating[1],math.floor((min(start[1],goal[1])-padding)/resolution)*resolution),
                    min(operating[2],math.ceil((max(start[0],goal[0])+padding)/resolution)*resolution),
                    min(operating[3],math.ceil((max(start[1],goal[1])+padding)/resolution)*resolution)]
        key=tuple(round(v,9) for v in window)
        if key not in seen:
            seen.add(key);yield window
        if window==list(operating):
            break


def _circle_plan(config,start,target=None,goal=None,start_yaw=None,contact=False,budget=None):
    settings=PlanningSettings.from_config(config)
    budget=budget or PlanningBudget(settings)
    destination=target['center'] if target is not None else goal
    if destination is None or not all(math.isfinite(v) for v in (*start,*destination)):
        raise ValueError('Invalid start/goal for planning')
    x0,y0,x1,y1=config['bounds']
    if not (x0<start[0]<x1 and y0<start[1]<y1):
        raise ValueError(f'Robot reference point outside operating area: {start}, bounds={config["bounds"]}')
    if goal is not None and not (x0<goal[0]<x1 and y0<goal[1]<y1):
        raise ValueError(f'Coordinate goal outside operating area: {goal}, bounds={config["bounds"]}')
    last_error=None;last_bounds=None
    for number,window in enumerate(planning_windows(config,start,destination,settings),1):
        budget.check()
        grid=GridMap(config,bounds=window,budget=budget)
        last_bounds=window
        print(f'Planning window {number}: bounds={window}, grid={grid.width}x{grid.height}, '
              f'operating_bounds={config["bounds"]}',flush=True)
        try:
            if target is None:
                route=plan(grid,start,goal,start_yaw=start_yaw)
            elif contact:
                route=plan_contact(grid,start,target,start_yaw=start_yaw)
            else:
                route=plan_to_object(grid,start,target,start_yaw=start_yaw)
            budget.check()
            details=grid.plan_stats.get(tuple(route))
            if details:
                print(f'PLAN_STATS: {details}',flush=True)
            # Budget is only for this command's planning, not later tracking.
            grid.budget=None
            if grid._planning_view is not None:
                grid._planning_view.budget=None
            return grid,route
        except ValueError as exc:
            last_error=exc
            print(f'Planning window {number}: {exc}; expanding if possible',flush=True)
    if last_bounds!=list(config['bounds']):
        raise PlanningLimitError(f'Planning window attempt limit reached; last_bounds={last_bounds}; '
                                 f'operating_bounds={config["bounds"]}; reason={last_error}')
    raise ValueError(f'No reachable approach/route within operating area {config["bounds"]}: {last_error}')


def adaptive_plan(config,start,target=None,goal=None,start_yaw=None,contact=False):
    def selected(item):
        print(f'PLANNER_SELECTED: {"hybrid" if hasattr(item[1],"motions") else "circle"}',flush=True)
        return item

    mode=config.get('planner_mode','circle')
    if mode not in ('circle','hybrid','auto'):raise ValueError('planner_mode must be circle, hybrid or auto')
    if mode=='circle' or 'footprint_bounds' not in config['robot']:
        if mode=='hybrid':raise ValueError('Hybrid planner requires robot.footprint_bounds')
        return selected(_circle_plan(config,start,target,goal,start_yaw,contact))
    if start_yaw is None or not math.isfinite(start_yaw):
        # Legacy callers without a known yaw cannot certify oriented motion.
        if mode=='auto' and start_yaw is None:return selected(_circle_plan(config,start,target,goal,start_yaw,contact))
        raise ValueError('Hybrid planning requires finite start_yaw')
    destination=target['center'] if target is not None else goal
    if destination is None or not all(math.isfinite(v) for v in (*start,*destination)):
        raise ValueError('Invalid start/goal for planning')
    x0,y0,x1,y1=config['bounds']
    if not (x0<start[0]<x1 and y0<start[1]<y1):
        raise ValueError(f'Robot reference point outside operating area: {start}, bounds={config["bounds"]}')
    if goal is not None and not (x0<goal[0]<x1 and y0<goal[1]<y1):
        raise ValueError(f'Coordinate goal outside operating area: {goal}, bounds={config["bounds"]}')
    settings=PlanningSettings.from_config(config);budget=PlanningBudget(settings)
    incumbent=None;connect_only=False
    if mode=='auto':
        try:
            incumbent=_circle_plan(config,start,target,goal,start_yaw,contact,
                                   PlanningSlice(budget,settings.timeout*.25,max(1,settings.max_expansions//4)))
            grid,route=incumbent
            length=sum(math.dist(a,b) for a,b in zip(route,route[1:]))
            connect_only=length<=1.5*max(.1,math.dist(start,route[-1]))
            if not connect_only:print('POSE_COMPARE: evaluating a long detour against an oriented route',flush=True)
        except (ValueError,PlanningLimitError) as exc:
            budget.check()
            print(f'POSE_FALLBACK: {exc}; preserving footprint and tracking margins',flush=True)
    from pose_planner import pose_plan
    try:
        candidate=pose_plan(config,(*start,start_yaw),target,goal,contact,budget,connect_only)
        if candidate is None:return selected(incumbent)
        if incumbent is None:return selected(candidate)
        from route_timing import estimate_time
        def duration(item):
            grid,route=item
            final=(math.atan2(target['center'][1]-route[-1][1],target['center'][0]-route[-1][0])
                   if target is not None and contact else None)
            duration=estimate_time(grid,route,start_yaw,final)
            if target is not None and contact:
                from contact_goal import target_ray_radius
                from motion_config import ContactSettings,MotionSettings
                distance=math.dist(route[-1],target['center'])
                travel=max(0.,distance-target_ray_radius(target,final+math.pi)-config['robot']['footprint_bounds'][2])
                duration+=travel/min(ContactSettings.from_config(config).max_speed,
                                     MotionSettings.from_config(config).max_linear_speed)
            return duration
        old,new=duration(incumbent),duration(candidate)
        print(f'POSE_COMPARE: circle={old:.2f}s, oriented={new:.2f}s (estimates)',flush=True)
        return selected(candidate if new<old*.98 else incumbent)
    except (ValueError,PlanningLimitError) as exc:
        if incumbent is None:raise
        print(f'POSE_COMPARE: keeping certified circle route; {exc}',flush=True)
        return selected(incumbent)
