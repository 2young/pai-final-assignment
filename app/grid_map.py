"""Single-source static field; conservatively occupied whole grid cells."""
import json
import math
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'maze.json'


def load_config(path=DEFAULT_CONFIG):
    return json.loads(Path(path).read_text())


class GridMap:
    def __init__(self, config):
        self.config = config
        self.bounds = config['bounds']
        self.resolution = config['resolution']
        self.padding = config['robot']['radius'] + config['robot']['margin']
        x0, y0, x1, y1 = self.bounds
        self.width = math.floor((x1-x0)/self.resolution)
        self.height = math.floor((y1-y0)/self.resolution)
        self.obstacles = config['walls'] + config['objects']
        self.blocked = {(x, y) for x in range(self.width) for y in range(self.height)
                        if not self.point_free(self.cell_center((x, y)),
                                               self.resolution/math.sqrt(2))}

    def point_free(self, point, extra=0):
        x, y = point
        if not all(math.isfinite(v) for v in point):
            return False
        p = self.padding + extra
        x0, y0, x1, y1 = self.bounds
        if not (x0+p < x < x1-p and y0+p < y < y1-p):
            return False
        for obj in self.obstacles:
            cx, cy = obj['center']
            sx, sy = obj['size'][:2]
            # Expanded AABB is conservative relative to a circular footprint.
            if abs(x-cx) <= sx/2+p and abs(y-cy) <= sy/2+p:
                return False
        return True

    def clearance(self, point):
        """Lower bound on footprint-to-obstacle distance, without safety margin."""
        x, y = point
        radius = self.config['robot']['radius']
        x0, y0, x1, y1 = self.bounds
        distances = [x-x0-radius, x1-x-radius, y-y0-radius, y1-y-radius]
        for obj in self.obstacles:
            cx, cy = obj['center']
            sx, sy = obj['size'][:2]
            distances.append(math.hypot(max(abs(x-cx)-sx/2, 0),
                                        max(abs(y-cy)-sy/2, 0))-radius)
        return min(distances)

    def cell(self, point):
        if not all(math.isfinite(v) for v in point):
            raise ValueError('Non-finite position')
        return tuple(math.floor((v-o)/self.resolution)
                     for v, o in zip(point, self.bounds[:2]))

    def cell_center(self, cell):
        return tuple(o+(v+0.5)*self.resolution for v, o in zip(cell, self.bounds[:2]))

    def free(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and cell not in self.blocked

    def segment_free(self, a, b):
        n = max(1, math.ceil(math.dist(a, b)/(self.resolution/4)))
        return all(self.free(self.cell((a[0]+(b[0]-a[0])*i/n,
                                        a[1]+(b[1]-a[1])*i/n))) for i in range(n+1))
