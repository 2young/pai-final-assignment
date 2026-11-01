"""8-connected A*, safe any-angle visibility edges, turn-prioritized search."""
import heapq
import math
from pose_provider import angle_difference

TURN_EPS = math.radians(1)
DIRECTIONS = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]


def octile(a,b):
    dx,dy=abs(a[0]-b[0]),abs(a[1]-b[1])
    return max(dx,dy)+(math.sqrt(2)-1)*min(dx,dy)


def astar(grid, start, goal):
    for name,cell in [('start',start),('goal',goal)]:
        if not grid.free(cell):
            raise ValueError(f'{name} outside field or occupied: {cell}')
    queue=[(octile(start,goal),0.,start)]
    costs,parents={start:0.},{}
    while queue:
        if hasattr(grid,'check_budget'):
            grid.check_budget(expansion=True)
        _,cost,current=heapq.heappop(queue)
        if cost!=costs[current]:
            continue
        if current==goal:
            path=[current]
            while current in parents:
                current=parents[current];path.append(current)
            return path[::-1]
        x,y=current
        for dx,dy in DIRECTIONS:
            nxt=(x+dx,y+dy)
            if not grid.free(nxt):
                continue
            if dx and dy and not (grid.free((x+dx,y)) and grid.free((x,y+dy))):
                continue
            updated=cost+math.hypot(dx,dy)
            if updated < costs.get(nxt,math.inf):
                costs[nxt],parents[nxt]=updated,current
                heapq.heappush(queue,(updated+octile(nxt,goal),updated,nxt))
    raise ValueError('No route to goal')


def route_score(route, start_yaw=None, final_yaw=None):
    headings=[math.atan2(b[1]-a[1],b[0]-a[0])
              for a,b in zip(route,route[1:]) if math.dist(a,b)>1e-8]
    length=sum(math.dist(a,b) for a,b in zip(route,route[1:]))
    angles=(([start_yaw] if start_yaw is not None else [])+headings+
            ([final_yaw] if final_yaw is not None else []))
    changes=[abs(angle_difference(b,a)) for a,b in zip(angles,angles[1:])]
    return sum(d>TURN_EPS for d in changes),length,sum(changes)


def _edge_cost(grid,length,angle):
    config=getattr(grid,'config',{})
    if config.get('path_following',{}).get('path_objective','turns')=='travel_time':
        speeds=config.get('navigation',{})
        return (length/speeds.get('max_linear_speed',.7)+.4*angle/speeds.get('max_angular_speed',.5),length,angle)
    return int(angle>TURN_EPS),length,angle


def selection_score(grid,route,start_yaw=None,final_yaw=None):
    turns,length,angle=route_score(route,start_yaw,final_yaw)
    first,_,_=_edge_cost(grid,length,angle)
    if getattr(grid,'config',{}).get('path_following',{}).get('path_objective','turns')=='turns':
        first=turns
    return first,length,angle


def _candidates(grid, points, start_yaw, final_yaw):
    """Retain A* bends for connectivity; add samples and orientation joins."""
    candidates=[points[0],points[-1]]
    distance=0.
    for i in range(1,len(points)-1):
        if i%16==0 and hasattr(grid,'check_budget'):
            grid.check_budget()
        a,b,c=points[i-1:i+2]
        u=(b[0]-a[0],b[1]-a[1]);v=(c[0]-b[0],c[1]-b[1])
        distance+=math.dist(a,b)
        if abs(u[0]*v[1]-u[1]*v[0])>1e-8 or distance>=1.0:
            candidates.append(b);distance=0.
    start,goal=points[0],points[-1]
    candidates.extend([(start[0],goal[1]),(goal[0],start[1])])
    if start_yaw is not None and final_yaw is not None:
        u=(math.cos(start_yaw),math.sin(start_yaw))
        v=(math.cos(final_yaw),math.sin(final_yaw))
        cross=u[0]*v[1]-u[1]*v[0]
        if abs(cross)>1e-8:
            delta=(goal[0]-start[0],goal[1]-start[1])
            t=(delta[0]*v[1]-delta[1]*v[0])/cross
            remaining=(u[0]*delta[1]-u[1]*delta[0])/cross
            if t>=0 and remaining>=0:
                candidates.append((start[0]+t*u[0],start[1]+t*u[1]))
    result=[];seen=set()
    for point in candidates:
        key=tuple(round(v,9) for v in point)
        if key not in seen and grid.segment_free(point,point):
            result.append(tuple(point));seen.add(key)
    if len(result)<2 or result[0]!=tuple(start) or result[1]!=tuple(goal):
        # Coincident points are handled by plan before candidate construction.
        raise ValueError('Unsafe start/goal connector')
    return result


def simplify_path(grid, points, start_yaw=None, final_yaw=None):
    """Exact lexicographic optimum in this finite visibility graph, not globally."""
    nodes=_candidates(grid,points,start_yaw,final_yaw)
    edges=[[] for _ in nodes]
    for i,a in enumerate(nodes):
        if hasattr(grid,'check_budget'):
            grid.check_budget()
        for j in range(i+1,len(nodes)):
            if hasattr(grid,'check_budget'):
                grid.check_budget()
            b=nodes[j]
            if grid.segment_free(a,b):
                length=math.dist(a,b)
                heading=math.atan2(b[1]-a[1],b[0]-a[0])
                edges[i].append((j,length,heading))
                edges[j].append((i,length,heading+math.pi))
    root=(-1,0);costs={root:(0,0.,0.)};parents={}
    queue=[(0,0.,0.,*root)]
    best=None;goal_state=None
    while queue:
        if hasattr(grid,'check_budget'):
            grid.check_budget(expansion=True)
        turns,length,angle,previous,current=heapq.heappop(queue)
        cost=(turns,length,angle);state=(previous,current)
        if cost!=costs.get(state):
            continue
        if best is not None and cost>=best:
            break
        incoming=(start_yaw if previous<0 else
                  math.atan2(nodes[current][1]-nodes[previous][1],nodes[current][0]-nodes[previous][0]))
        if current==1:
            delta=0. if final_yaw is None or incoming is None else abs(angle_difference(final_yaw,incoming))
            increment=_edge_cost(grid,0.,delta)
            complete=tuple(a+b for a,b in zip(cost,increment))
            if best is None or complete<best:
                best,goal_state=complete,state
            continue
        for nxt,segment,heading in edges[current]:
            if nxt==previous or nxt==0:
                continue
            delta=0. if incoming is None else abs(angle_difference(heading,incoming))
            increment=_edge_cost(grid,segment,delta)
            updated=tuple(a+b for a,b in zip(cost,increment))
            next_state=(current,nxt)
            if updated<costs.get(next_state,(math.inf,math.inf,math.inf)):
                costs[next_state],parents[next_state]=updated,state
                heapq.heappush(queue,(*updated,*next_state))
    if goal_state is None:
        raise ValueError('No safe visibility route')
    route=[];state=goal_state
    while True:
        route.append(nodes[state[1]])
        if state==root:
            break
        state=parents[state]
    return route[::-1]


def plan(grid, start, goal, start_yaw=None, final_yaw=None):
    if any(a is not None and not math.isfinite(a) for a in (start_yaw,final_yaw)):
        raise ValueError('Non-finite planning heading')
    planning=grid.planning_view()
    cells=astar(planning,planning.cell(start),planning.cell(goal))
    if math.dist(start,goal)<1e-8:
        if not planning.segment_free(start,goal):
            raise ValueError('Unsafe path connector')
        return [tuple(start),tuple(goal)]
    points=[tuple(start)]
    for point in [planning.cell_center(c) for c in cells]+[tuple(goal)]:
        if math.dist(points[-1],point)>1e-8:
            points.append(point)
    route=simplify_path(planning,points,start_yaw,final_yaw)
    if not all(planning.segment_free(a,b) for a,b in zip(route,route[1:])):
        raise ValueError('Unsafe path connector')
    if hasattr(grid,'plan_stats'):
        grid.plan_stats[tuple(route)]={
            'raw_cells':len(cells),
            'raw_length':sum(math.dist(a,b)*planning.resolution for a,b in zip(cells,cells[1:])),
            'final_length':sum(math.dist(a,b) for a,b in zip(route,route[1:])),
            'waypoints':len(route)}
    return route
