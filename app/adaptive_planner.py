"""Bounded expanding search windows inside a fixed robot operating area."""
import math
from grid_map import GridMap
from planning_limits import PlanningBudget,PlanningSettings,PlanningLimitError
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


def adaptive_plan(config,start,target=None,goal=None,start_yaw=None,contact=False):
    settings=PlanningSettings.from_config(config)
    budget=PlanningBudget(settings)
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
