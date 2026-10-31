"""Single-source static field; conservatively occupied whole grid cells."""
import json
import math
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'maze.json'


def load_config(path=DEFAULT_CONFIG):
    return json.loads(Path(path).read_text())


class GridMap:
    def __init__(self, config, tracking_margin=0):
        self.config = config
        self.bounds = config['bounds']
        self.resolution = config['resolution']
        self.padding = config['robot']['radius'] + config['robot']['margin'] + tracking_margin
        self.tracking_margin = tracking_margin
        self._planning_view = None
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

    def planning_view(self):
        """An extra 10cm for tracking; runtime safety retains its own grid."""
        if self.tracking_margin:
            return self
        if self._planning_view is None:
            self._planning_view = GridMap(self.config, tracking_margin=.10)
        return self._planning_view

    def segment_cells(self, a, b):
        """Supercover: all closed cells touched, including exact edges/corners."""
        if not all(math.isfinite(v) for v in (*a,*b)):
            raise ValueError('Non-finite segment')
        start = tuple((v-o)/self.resolution for v,o in zip(a,self.bounds[:2]))
        end = tuple((v-o)/self.resolution for v,o in zip(b,self.bounds[:2]))
        times = [0.,1.]
        for p,q in zip(start,end):
            if abs(q-p)>1e-12:
                for k in range(math.floor(min(p,q)),math.ceil(max(p,q))+1):
                    t=(k-p)/(q-p)
                    if 0<t<1:
                        times.append(t)
        events=[]
        for t in sorted(times):
            if not events or t-events[-1]>1e-12:
                events.append(t)
        samples=events+[(p+q)/2 for p,q in zip(events,events[1:])]
        covered=set()
        for t in samples:
            x,y=(p+(q-p)*t for p,q in zip(start,end))
            def indices(value):
                n=round(value)
                return (n-1,n) if abs(value-n)<1e-9 else (math.floor(value),)
            for i in indices(x):
                for j in indices(y):
                    covered.add((i,j))
        return covered

    def segment_free(self, a, b):
        try:
            if not self.free(self.cell(a)) or not self.free(self.cell(b)):
                return False
            return all(self.free(c) for c in self.segment_cells(a,b))
        except ValueError:
            return False
