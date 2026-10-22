import math
import time
import json
from pathlib import Path


class Navigator:
    def __init__(self, motor, lidar, odometry, semantic_map, settings, cameras=None):
        self.motor, self.lidar, self.odometry = motor, lidar, odometry
        self.map, self.settings, self.cameras = semantic_map, settings, cameras
        self._last_trace = 0
        self._last_integrated_scan = None
        self.last_goal = None
        self.phase_deadline = None
        self.localizer = None
        if settings.get('scan_localization'):
            from src.control.scan_localization import ScanLocalization
            self.localizer = ScanLocalization(settings)
        self.radius = math.hypot(settings['robot_width_m'], settings['robot_length_m'])/2+settings['clearance_m']

    def sense(self):
        if self.phase_deadline is not None and time.monotonic()>=self.phase_deadline:
            self.motor.stop_all()
            raise RuntimeError('Outward phase time exhausted; reserve return time')
        if self.motor.fault:
            raise RuntimeError(self.motor.fault)
        scan = self.lidar.get_fresh_scan(timeout=0.1)
        if not scan:
            self.motor.stop_all()
            raise RuntimeError('Missing or stale lidar; navigation aborted')
        pose = self.odometry.get_pose_meters()
        if self.cameras is not None and len(self.cameras.get_frames()) != 3:
            self.motor.stop_all()
            raise RuntimeError('A camera is missing/stale')
        if self.localizer is not None:
            try:
                pose = self.localizer.update(scan, self.lidar.last_scan_at, pose)
            except Exception:
                self.motor.stop_all()
                raise
        self.map.update_robot_pose(*pose)
        stamp = self.lidar.last_scan_at
        if stamp != self._last_integrated_scan:
            self.map.grid.integrate_scan(scan, pose, self.settings['lidar_front_deg'], self.settings['lidar_angle_sign'])
            self._last_integrated_scan = stamp
        if time.monotonic()-self._last_trace > .5:
            path=Path('data/navigation_trace.jsonl');path.parent.mkdir(parents=True,exist_ok=True)
            with path.open('a',encoding='utf-8') as out:
                out.write(json.dumps({'time':time.time(),'pose':pose,'goal':self.last_goal,
                           'speed_command':self.motor.last_speed_command,
                           'scan_alignment':self.localizer.last_result if self.localizer else None})+'\n')
            self._last_trace=time.monotonic()
        return scan, pose

    def rotate_to(self, target, timeout=45):
        deadline = time.monotonic()+timeout
        try:
            while time.monotonic() < deadline:
                scan, pose = self.sense()
                error = math.atan2(math.sin(target-pose[2]), math.cos(target-pose[2]))
                if abs(error) < 0.08:
                    return
                if min(d/1000 for q, a, d in scan if q > 0) < self.radius:
                    raise RuntimeError('Insufficient clearance to rotate')
                speed = min(self.settings['turn_speed'], max(20, int(80*abs(error))))
                self.motor.rotate_left(speed if error > 0 else -speed)
                time.sleep(0.05)
            raise RuntimeError('Rotation timeout')
        finally:
            self.motor.stop_all()

    def _follow_segment(self, goal, timeout=20):
        deadline = time.monotonic()+timeout
        try:
            while time.monotonic() < deadline:
                scan, pose = self.sense()
                dx, dy = goal[0]-pose[0], goal[1]-pose[1]
                if math.hypot(dx, dy) < 0.05:
                    return
                heading = math.atan2(dy, dx)
                error = math.atan2(math.sin(heading-pose[2]), math.cos(heading-pose[2]))
                if abs(error) > 0.12:
                    self.motor.stop_all()
                    self.rotate_to(heading,timeout=max(0,deadline-time.monotonic()))
                    continue
                front = self.settings['lidar_front_deg']
                zone = [d/1000 for q, a, d in scan if q > 0 and abs((a-front+180) % 360-180) <= 35]
                if len(zone) < 5 or min(zone) < self.settings['stop_distance_m']:
                    raise RuntimeError('Obstacle or unobserved forward sector')
                # Revalidate remaining straight segment against updated occupancy.
                blocked = self.map.grid.blocked(self.radius)
                steps = max(1, int(math.hypot(dx, dy)/self.map.grid.resolution)+1)
                for step in range(steps+1):
                    point = pose[0]+dx*step/steps, pose[1]+dy*step/steps
                    if not self.map.grid.is_position_clear(point,self.radius):
                        raise RuntimeError('Route blocked by new observations')
                speed = min(self.settings['drive_speed'], max(60, int(math.hypot(dx, dy)*700)))
                self.motor.move_forward_with_arc('left' if error > 0 else 'right', speed,
                                                min(speed//3, int(abs(error)*speed)))
                time.sleep(0.05)
            raise RuntimeError('Translation timeout')
        finally:
            self.motor.stop_all()

    @staticmethod
    def corners(route):
        if len(route) <= 2:
            return route[1:]
        corners = []
        previous_direction = None
        for index in range(1, len(route)):
            dx, dy = route[index][0]-route[index-1][0], route[index][1]-route[index-1][1]
            direction = (round(dx, 4), round(dy, 4))
            if previous_direction is not None and direction != previous_direction:
                corners.append(route[index-1])
            previous_direction = direction
        corners.append(route[-1])
        return corners

    def line_clear(self, start, end):
        count=max(1,int(math.dist(start,end)/(self.map.grid.resolution/2))+1)
        return all(self.map.grid.is_position_clear((start[0]+(end[0]-start[0])*i/count,
                    start[1]+(end[1]-start[1])*i/count),self.radius) for i in range(count+1))

    def shortcut_route(self, route, start, goal):
        # Remove grid zigzags only when the actual swept circle is observed free.
        points=list(route[1:])+[tuple(goal)]
        result=[];position=tuple(start)
        while points:
            chosen=None
            for index in range(len(points)-1,-1,-1):
                if self.line_clear(position,points[index]):
                    chosen=index;break
            if chosen is None:
                raise RuntimeError('Route blocked by new observations')
            position=points[chosen];result.append(position);points=points[chosen+1:]
        return result

    def navigate(self, goal, timeout=120):
        self.last_goal = list(goal)
        deadline = time.monotonic()+timeout
        last_error = None
        try:
            while time.monotonic() < deadline:
                _, pose = self.sense()
                if math.hypot(goal[0]-pose[0], goal[1]-pose[1]) < 0.08:
                    return True
                route = self.map.grid.route(pose[:2], goal, self.radius)
                if not route:
                    self.motor.stop_all()
                    last_error = f'No observed collision-free route: pose={pose}, goal={goal}'
                    time.sleep(.2)
                    continue
                try:
                    for point in self.shortcut_route(route,pose[:2],goal):
                        self._follow_segment(point, timeout=min(60, max(0, deadline-time.monotonic())))
                except RuntimeError as exc:
                    last_error = str(exc)
                    if 'Obstacle' not in last_error and 'Route blocked' not in last_error:
                        raise
                    # A* replans with the updated lidar map. No blind sidestep/reverse.
                    time.sleep(0.1)
                    continue
            raise RuntimeError(last_error or 'Navigation deadline exceeded')
        finally:
            self.motor.stop_all()
