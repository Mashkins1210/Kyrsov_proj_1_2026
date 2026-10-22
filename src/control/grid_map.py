"""Observed occupancy grid, inflated A* routes, and camera/lidar observations."""
import heapq
import math
import numpy as np


class GridMap:
    def __init__(self, resolution=0.05, size=4.0):
        self.resolution = resolution
        self.size = size
        self.n = int(round(size/resolution))
        self.log_odds = np.zeros((self.n, self.n), dtype=float)
        self.observed = np.zeros((self.n, self.n), dtype=bool)

    def index(self, x, y):
        return int(math.floor(y/self.resolution)), int(math.floor(x/self.resolution))

    def inside(self, cell):
        return 0 <= cell[0] < self.n and 0 <= cell[1] < self.n

    def point(self, cell):
        return (cell[1]+0.5)*self.resolution, (cell[0]+0.5)*self.resolution

    def integrate_scan(self, scan, pose, lidar_yaw_deg=180.0, angle_sign=-1):
        x, y, heading = pose
        start = self.index(x, y)
        if not self.inside(start):
            raise ValueError('Robot pose outside the arena')
        # The robot's currently occupied footprint is known traversable; the
        # lidar minimum range leaves this disk unmeasured otherwise.
        for row in range(max(0,start[0]-3), min(self.n,start[0]+4)):
            for col in range(max(0,start[1]-3), min(self.n,start[1]+4)):
                px, py = self.point((row,col))
                if math.hypot(px-x,py-y) <= 0.14:
                    self.observed[row,col] = True
                    self.log_odds[row,col] = -5
        for quality, angle, distance in scan:
            if quality <= 0 or not math.isfinite(distance) or not 150 <= distance <= 12000:
                continue
            bearing = heading + angle_sign*math.radians(angle-lidar_yaw_deg)
            length = distance/1000
            # Do not invent free space beyond a measured beam endpoint.
            steps = max(1, int(length/(self.resolution/2)))
            last = None
            for step in range(steps):
                t = step/steps
                cell = self.index(x+t*length*math.cos(bearing), y+t*length*math.sin(bearing))
                if not self.inside(cell):
                    break
                if cell != last:
                    self.observed[cell] = True
                    self.log_odds[cell] = max(-5, self.log_odds[cell]-0.3)
                    last = cell
            end = self.index(x+length*math.cos(bearing), y+length*math.sin(bearing))
            if self.inside(end):
                self.observed[end] = True
                self.log_odds[end] = min(5, self.log_odds[end]+0.9)

    def blocked(self, radius):
        # Unknown space is blocked in execution routes.
        occupied = ~self.observed | (self.log_odds >= 0.5)
        cells = int(math.ceil(radius/self.resolution))
        padded = np.pad(occupied, cells, constant_values=True)
        blocked = np.zeros_like(occupied)
        for dy in range(-cells, cells+1):
            for dx in range(-cells, cells+1):
                if math.hypot(dx, dy)*self.resolution <= radius+self.resolution/2:
                    blocked |= padded[cells+dy:cells+dy+self.n, cells+dx:cells+dx+self.n]
        return blocked

    def is_position_clear(self, point, radius):
        # Check the continuous circle against occupied/unknown cell rectangles.
        # Rounding the robot to a cell center can shift its clearance by 25 mm.
        x,y=point
        if min(x,y,self.size-x,self.size-y)<radius:
            return False
        r,c=self.index(x,y);reach=int(math.ceil(radius/self.resolution))+1
        for rr in range(max(0,r-reach),min(self.n,r+reach+1)):
            for cc in range(max(0,c-reach),min(self.n,c+reach+1)):
                if self.observed[rr,cc] and self.log_odds[rr,cc]<.5:
                    continue
                dx=max(cc*self.resolution-x,0,x-(cc+1)*self.resolution)
                dy=max(rr*self.resolution-y,0,y-(rr+1)*self.resolution)
                if math.hypot(dx,dy)<=radius:
                    return False
        return True

    def route(self, start_point, goal_point, radius):
        start, goal = self.index(*start_point), self.index(*goal_point)
        blocked = self.blocked(radius)
        if not self.inside(start) or not self.inside(goal):
            return None
        if not self.is_position_clear(start_point,radius) or not self.is_position_clear(goal_point,radius):
            return None
        # Endpoints are continuous positions, not the rounded grid centers.
        blocked[start]=False
        blocked[goal]=False
        queue = [(0, start)]
        cost = {start: 0}
        parent = {}
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                path = [current]
                while current != start:
                    current = parent[current]
                    path.append(current)
                return [self.point(c) for c in reversed(path)]
            for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                neighbour = current[0]+dy, current[1]+dx
                if not self.inside(neighbour) or blocked[neighbour]:
                    continue
                value = cost[current]+1
                if value < cost.get(neighbour, float('inf')):
                    cost[neighbour] = value
                    parent[neighbour] = current
                    h = abs(goal[0]-neighbour[0])+abs(goal[1]-neighbour[1])
                    heapq.heappush(queue, (value+h, neighbour))
        return None

    def approach(self, start_point, object_point, radius, standoff=0.35):
        candidates = []
        for angle in range(0, 360, 30):
            a = math.radians(angle)
            p = (object_point[0]+standoff*math.cos(a), object_point[1]+standoff*math.sin(a))
            route = self.route(start_point, p, radius)
            if route:
                candidates.append(route)
        return min(candidates, key=len) if candidates else None

    def save_data(self):
        return {'resolution': self.resolution, 'size': self.size,
                'log_odds': self.log_odds.tolist(), 'observed': self.observed.tolist()}

    @classmethod
    def from_data(cls, data):
        grid = cls(data['resolution'], data['size'])
        grid.log_odds = np.asarray(data['log_odds'], dtype=float)
        grid.observed = np.asarray(data['observed'], dtype=bool)
        if grid.log_odds.shape != (grid.n, grid.n) or grid.observed.shape != (grid.n, grid.n) or not np.isfinite(grid.log_odds).all():
            raise ValueError('Invalid occupancy map')
        return grid
