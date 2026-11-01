"""Resolve registered metadata only. This is not camera-based recognition."""
import math
import re
from path_planner import plan,route_score,selection_score


def resolve_target(text, config):
    matches = [o for o in config['objects'] if any(a in text.lower() for a in o['aliases'])]
    # Reject a mixture of known and unknown colors as well as ambiguous matches.
    colors = re.findall('赤|青|緑|黄|紫|白|黒|オレンジ|ピンク', text)
    if len(matches) != 1 or any(c not in matches[0]['aliases'] for c in colors):
        raise ValueError('Target unknown or ambiguous; specify one registered color/name')
    return matches[0]


def approach_candidates(grid,obj,start=None,start_yaw=None,extras=(.35,)):
    """Axis, diagonal and pose-aligned rays outside the inflated object AABB."""
    cx, cy = obj['center']
    sx, sy = obj['size'][:2]
    angles=[math.pi,0,-math.pi/2,math.pi/2,
            -3*math.pi/4,-math.pi/4,math.pi/4,3*math.pi/4]
    if start is not None and math.dist(start,(cx,cy))>1e-8:
        angles.append(math.atan2(start[1]-cy,start[0]-cx))
    if start_yaw is not None:
        angles.append(start_yaw+math.pi)
    seen=set()
    for angle in angles:
        key=round(angle%(2*math.pi),8)
        if key in seen:
            continue
        seen.add(key)
        ux,uy=math.cos(angle),math.sin(angle)
        for extra in extras:
            distance=min(half/abs(u) if abs(u)>1e-10 else math.inf
                         for half,u in [(sx/2+grid.padding+extra,ux),
                                        (sy/2+grid.padding+extra,uy)])
            yield (cx+ux*distance,cy+uy*distance),math.atan2(-uy,-ux)


def plan_to_object(grid, start, obj, start_yaw=None):
    routes = []
    for goal,_ in approach_candidates(grid,obj,start,start_yaw):
        try:
            route = plan(grid, start, goal,start_yaw=start_yaw)
            routes.append(route)
        except ValueError:
            continue
    if not routes:
        raise ValueError('No reachable approach point')
    return min(routes, key=lambda r: selection_score(grid,r,start_yaw))
