"""Snapshot live registered objects into a planning map; guard against changes."""
import copy
import math
import time
from pose_provider import PoseProvider, angle_difference

MOVE_TOLERANCE = 0.05
ANGLE_TOLERANCE = math.radians(3)
HEIGHT_TOLERANCE = 0.05
UPRIGHT_TOLERANCE = 1e-4


def runtime_config(config, poses):
    """Keep the source registry intact; replace each object's pose and XY bounds."""
    result = copy.deepcopy(config)
    x0, y0, x1, y1 = config['bounds']
    for obj in result['objects']:
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
        if not (x0 < pose.x-width/2 and pose.x+width/2 < x1 and
                y0 < pose.y-height/2 and pose.y+height/2 < y1):
            raise ValueError(f"{obj['id']}: object outside field")
        obj['center'] = [pose.x, pose.y]
        obj['size'] = [width, height, sz]
        obj['yaw'] = pose.yaw
    for i, obj in enumerate(result['objects']):
        for other in result['walls'] + result['objects'][:i]:
            if all(abs(a-b) <= (sa+sb)/2 for a,b,sa,sb in
                   zip(obj['center'], other['center'], obj['size'], other['size'])):
                raise ValueError(f"{obj['id']}: overlaps {other['id']} (conservative bounds)")
    return result


class ObjectTracker:
    def __init__(self, node, config):
        self.node, self.config = node, config
        self.providers = {o['id']: PoseProvider(node, o['id']) for o in config['objects']}
        self.baseline = None

    def snapshot(self, timeout=10):
        import rclpy
        requested = time.monotonic()
        deadline = requested+timeout
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            # Require a new observation on every command, not a previous snapshot.
            if all(p.state is not None and p.received >= requested
                   for p in self.providers.values()):
                poses = {name: p.current_state() for name,p in self.providers.items()}
                live = runtime_config(self.config, poses)
                self.baseline = poses
                for obj in live['objects']:
                    print(f"Live object: {obj['id']}, center={obj['center']}, "
                          f"yaw={obj['yaw']:.3f}, AABB={obj['size'][:2]}", flush=True)
                return live
            if time.monotonic() >= deadline:
                missing = [name for name,p in self.providers.items()
                           if p.state is None or p.received < requested]
                raise RuntimeError(f'No fresh object world pose: {missing}; restart maze/bridge')

    def assert_unchanged(self):
        if self.baseline is None:
            raise RuntimeError('Object snapshot not initialized')
        for name, original in self.baseline.items():
            current = self.providers[name].current_state()
            translation = math.dist((original.x, original.y, original.z),
                                    (current.x, current.y, current.z))
            rotation = max(abs(angle_difference(getattr(current, axis), getattr(original, axis)))
                           for axis in ('roll', 'pitch', 'yaw'))
            if translation > MOVE_TOLERANCE or rotation > ANGLE_TOLERANCE:
                raise RuntimeError(f'{name}: object moved during navigation; issue command again')
