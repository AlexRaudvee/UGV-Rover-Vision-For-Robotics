"""Eight-connected A* on an already inflated occupancy grid."""
import heapq
import math


def plan(data, width, height, resolution, origin, start, goal, allow_unknown=True):
    if width <= 0 or height <= 0 or resolution <= 0 or len(data) != width * height:
        raise ValueError('Invalid occupancy grid')
    def cell(point):
        return (math.floor((point[0]-origin[0])/resolution),
                math.floor((point[1]-origin[1])/resolution))
    def valid(p):
        x, y = p
        if not (0 <= x < width and 0 <= y < height):
            return False
        v = data[y*width+x]
        return v < 50 and (allow_unknown or v >= 0)
    start_cell, end = cell(start), cell(goal)
    if not valid(start_cell):
        return [], 'start is occupied or outside the field'
    if not valid(end):
        return [], 'goal is occupied or outside the field'
    open_set = [(0., 0., start_cell)]
    costs = {start_cell: 0.}
    parent = {}
    while open_set:
        _, cost, current = heapq.heappop(open_set)
        if cost > costs[current]:
            continue
        if current == end:
            cells = [current]
            while current != start_cell:
                current = parent[current]
                cells.append(current)
            cells.reverse()
            centres = [(origin[0]+(x+.5)*resolution, origin[1]+(y+.5)*resolution) for x,y in cells]
            # Include endpoints and cell centres: each segment remains in its cell.
            return [tuple(start)] + centres + [tuple(goal)], ''
        x, y = current
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nxt = (x+dx, y+dy)
            if not valid(nxt) or (dx and dy and (not valid((x+dx,y)) or not valid((x,y+dy)))):
                continue
            new_cost = cost + (math.sqrt(2.) if dx and dy else 1.)
            if new_cost < costs.get(nxt, math.inf):
                costs[nxt] = new_cost
                parent[nxt] = current
                heuristic = math.hypot(end[0]-nxt[0], end[1]-nxt[1])
                heapq.heappush(open_set, (new_cost+heuristic, new_cost, nxt))
    return [], 'no path to goal'
