"""Four-connected A* with Manhattan heuristic and explicit failure reasons."""
import heapq
import math


def astar(grid, start, goal):
    for name, cell in [('start', start), ('goal', goal)]:
        if not grid.free(cell):
            raise ValueError(f'{name} outside field or occupied: {cell}')
    queue = [(0, start)]
    costs, parents = {start: 0}, {}
    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            path = [current]
            while current in parents:
                current = parents[current]
                path.append(current)
            return path[::-1]
        x, y = current
        for nxt in [(x+1,y), (x-1,y), (x,y+1), (x,y-1)]:
            cost = costs[current]+1
            if grid.free(nxt) and cost < costs.get(nxt, math.inf):
                costs[nxt], parents[nxt] = cost, current
                h = abs(nxt[0]-goal[0])+abs(nxt[1]-goal[1])
                heapq.heappush(queue, (cost+h, nxt))
    raise ValueError('No route to goal')


def plan(grid, start, goal):
    cells = astar(grid, grid.cell(start), grid.cell(goal))
    points = [start] + [grid.cell_center(c) for c in cells] + [goal]
    # Only collapse collinear cells; retain the checked start connector.
    result = [points[0]]
    for i in range(1, len(points)-1):
        a, b, c = result[-1], points[i], points[i+1]
        cross = (b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0])
        if abs(cross) > 1e-8 or not grid.segment_free(a, c):
            result.append(b)
    result.append(goal)
    if not all(grid.segment_free(a, b) for a, b in zip(result, result[1:])):
        raise ValueError('Unsafe path connector')
    return result
