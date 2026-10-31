"""Snapshot all registered walls and targets; guard against scene changes."""
import copy
import math
import time
from pose_provider import PoseProvider, angle_difference
from field_geometry import intersects_operating_area
from ros_feedback import spin_some

MOVE_TOLERANCE = 0.05
ANGLE_TOLERANCE = math.radians(3)
HEIGHT_TOLERANCE = 0.05
UPRIGHT_TOLERANCE = 1e-4
SNAPSHOT_SKEW = .15


def scene_models(config):
    models=config['walls']+config['objects']
    ids=[o['id'] for o in models]
    if len(set(ids))!=len(ids) or any(not name or name=='vehicle_blue' for name in ids):
        raise ValueError('Scene model IDs must be unique and distinct from vehicle_blue')
    return models


def runtime_config(config, poses):
    """Replace all registered poses; never fill missing observations from JSON."""
    missing=[o['id'] for o in scene_models(config) if o['id'] not in poses]
    if missing:
        raise RuntimeError(f'No fresh object world pose: {missing}; all scene models required')
    result = copy.deepcopy(config)
    x0, y0, x1, y1 = config['bounds']
    policy=config.get('boundary_policy','whole_object')
    if policy not in ('whole_object','reachable_surface'):
        raise ValueError('boundary_policy must be whole_object or reachable_surface')
    target_ids={o['id'] for o in result['objects']}
    for obj in scene_models(result):
        pose = poses[obj['id']]
        if not all(math.isfinite(v) for v in
                   (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw)):
            raise ValueError(f"{obj['id']}: non-finite object pose")
        sx, sy, sz = obj['size']
        obj['body_size'] = list(obj['size'])
        if (abs(pose.z-sz/2) > HEIGHT_TOLERANCE or
                abs(pose.roll) > UPRIGHT_TOLERANCE or abs(pose.pitch) > UPRIGHT_TOLERANCE):
            raise ValueError(f"{obj['id']}: place object upright on the ground")
        c, s = abs(math.cos(pose.yaw)), abs(math.sin(pose.yaw))
        # A world-axis AABB contains the rotated box, also for approach planning.
        width, height = sx*c+sy*s, sx*s+sy*c
        contained=(x0 < pose.x-width/2 and pose.x+width/2 < x1 and
                   y0 < pose.y-height/2 and pose.y+height/2 < y1)
        intersects=intersects_operating_area((pose.x,pose.y),(sx,sy),pose.yaw,config['bounds'])
        # Walls may cross or leave the field: keep their true geometry so
        # inflation and boundary/contact checks can still account for them.
        if obj['id'] in target_ids and ((policy=='whole_object' and not contained) or not intersects):
            raise ValueError(f"{obj['id']}: object outside operating area; center=({pose.x:.3f},"
                             f"{pose.y:.3f}), size=({width:.3f},{height:.3f}), bounds={config['bounds']}")
        obj['center'] = [pose.x, pose.y]
        obj['size'] = [width, height, sz]
        obj['yaw'] = pose.yaw
        obj['crosses_boundary'] = not contained
    for i, obj in enumerate(result['objects']):
        for other in result['walls'] + result['objects'][:i]:
            if all(abs(a-b) <= (sa+sb)/2 for a,b,sa,sb in
                   zip(obj['center'], other['center'], obj['size'], other['size'])):
                raise ValueError(f"{obj['id']}: overlaps {other['id']} (conservative bounds)")
    return result


class ObjectTracker:
    def __init__(self, node, config):
        self.node, self.config = node, config
        self.providers = {o['id']: PoseProvider(node, o['id']) for o in scene_models(config)}
        self.baseline = None

    def snapshot(self, timeout=10):
        self.baseline=None
        requested = time.monotonic()
        deadline = requested+timeout
        while True:
            spin_some(self.node, timeout_sec=0.05)
            # Require a new observation on every command, not a previous snapshot.
            arrivals=[p.received for p in self.providers.values()]
            complete=all(p.state is not None and p.received >= requested
                         for p in self.providers.values())
            coherent=not arrivals or max(arrivals)-min(arrivals)<=SNAPSHOT_SKEW
            if complete and coherent:
                poses = {name: p.current_state() for name,p in self.providers.items()}
                live = runtime_config(self.config, poses)
                self.baseline = poses
                for obj in scene_models(live):
                    kind='object' if obj in live['objects'] else 'obstacle'
                    print(f"Live {kind}: {obj['id']}, center={obj['center']}, "
                          f"yaw={obj['yaw']:.3f}, AABB={obj['size'][:2]}, "
                          f"crosses_boundary={obj['crosses_boundary']}", flush=True)
                return live
            if time.monotonic() >= deadline:
                missing = [name for name,p in self.providers.items()
                           if p.state is None or p.received < requested]
                if not missing:
                    raise RuntimeError('Scene pose observations are not time-coherent; check Gazebo/bridge load')
                raise RuntimeError(f'No fresh object world pose: {missing}; restart maze/bridge')

    def assert_unchanged(self,max_age=None):
        if self.baseline is None:
            raise RuntimeError('Object snapshot not initialized')
        for name, original in self.baseline.items():
            current = self.providers[name].current_state()
            if max_age is not None and time.monotonic()-self.providers[name].received>max_age:
                raise RuntimeError(f'{name}: contact object pose missing or stale')
            translation = math.dist((original.x, original.y, original.z),
                                    (current.x, current.y, current.z))
            rotation = max(abs(angle_difference(getattr(current, axis), getattr(original, axis)))
                           for axis in ('roll', 'pitch', 'yaw'))
            if translation > MOVE_TOLERANCE or rotation > ANGLE_TOLERANCE:
                raise RuntimeError(f'{name}: object moved during navigation; issue command again')
