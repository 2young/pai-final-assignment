"""Resolve registered metadata only. This is not camera-based recognition."""
import math
import re
from path_planner import plan


def resolve_target(text, config):
    matches = [o for o in config['objects'] if any(a in text.lower() for a in o['aliases'])]
    # Reject a mixture of known and unknown colors as well as ambiguous matches.
    colors = re.findall('赤|青|緑|黄|紫|白|黒|オレンジ|ピンク', text)
    if len(matches) != 1 or any(c not in matches[0]['aliases'] for c in colors):
        raise ValueError('Target unknown or ambiguous; specify one registered color/name')
    return matches[0]


def plan_to_object(grid, start, obj):
    cx, cy = obj['center']
    sx, sy = obj['size'][:2]
    gap = grid.padding + 0.35
    candidates = [(cx-sx/2-gap,cy), (cx+sx/2+gap,cy),
                  (cx,cy-sy/2-gap), (cx,cy+sy/2+gap)]
    routes = []
    for goal in candidates:
        try:
            route = plan(grid, start, goal)
            routes.append(route)
        except ValueError:
            continue
    if not routes:
        raise ValueError('No reachable approach point')
    return min(routes, key=lambda r: sum(math.dist(a,b) for a,b in zip(r,r[1:])))
