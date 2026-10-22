"""Evacuation mission and training entry point. Motion requires --drive."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.control.llm_planner import LLMPlanner
from src.control.semantic_map import SemanticMap
from src.hardware.power_status import read_status


def load_settings(path):
    settings = json.loads(Path(path).read_text(encoding='utf-8'))
    for key in ('robot_width_m', 'robot_length_m', 'clearance_m', 'wheel_diameter_mm', 'wheelbase_mm', 'stop_distance_m'):
        value = settings[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid {key}')
    return settings


def validate_drive(settings):
    for key in ('geometry_verified', 'camera_geometry_verified', 'lidar_geometry_verified', 'localization_verified'):
        if settings.get(key) is not True:
            if key != 'localization_verified' or not settings.get('rehearsal') or not settings.get('scan_localization'):
                raise ValueError(f'Motion requires measured calibration: {key}')
    cell = settings.get('start_cell')
    if not isinstance(cell, list) or len(cell) != 2 or any(type(v) is not int or not 0 <= v < 5 for v in cell):
        raise ValueError('Specify start_cell [x/column, y/row], indices 0..4')
    if type(settings.get('start_heading_deg')) not in (int, float) or not math.isfinite(settings['start_heading_deg']):
        raise ValueError('Specify measured start heading')
    yaws = settings.get('camera_yaws_deg')
    if not isinstance(yaws, list) or len(yaws) != 3 or sorted(yaws) != [-90, 0, 90]:
        raise ValueError('Identify front/left/right camera indices before motion')
    if any(type(settings.get(k)) is not int or not 1 <= settings[k] <= 250 for k in ('drive_speed', 'turn_speed')):
        raise ValueError('Uncalibrated or excessive drive speed')


class MissionApp:
    def __init__(self, settings, semantic_map, report_path):
        from src.hardware.motor_controller import MotorController, LEFT_MOTORS, RIGHT_MOTORS
        from src.hardware.lidar_controller import LidarController
        from src.perception.camera_controller import CameraController
        from src.control.odometry import Odometry
        from src.control.navigation import Navigator
        from src.perception.semantic_observer import SemanticObserver
        validate_drive(settings)
        self.settings, self.map, self.report_path = settings, semantic_map, Path(report_path)
        self.resources = []
        self.report = {'success': False, 'events': [], 'victim': None,
                       'rehearsal': bool(settings.get('rehearsal')),
                       'calibration': {k: settings.get(k) for k in ('localization_verified','angular_calibration_verified')},
                       'operator_accepts_power_faults': bool(settings.get('operator_accepts_power_faults'))}
        self.mission_deadline = settings.get('absolute_deadline')
        self.guard_stop = threading.Event()
        try:
            self.motor = MotorController(); self.resources.append(self.motor)
            self.lidar = LidarController(); self.resources.append(self.lidar)
            self.camera = CameraController(devices=settings.get('camera_devices')); self.resources.append(self.camera)
            self.odom = Odometry(self.motor, LEFT_MOTORS, RIGHT_MOTORS, settings['wheel_diameter_mm'], settings['wheelbase_mm'], angular_scale=settings.get('odometry_angular_scale',1.0))
            self.resources.append(self.odom)
            col, row = settings['start_cell']
            start = ((col+0.5)*0.8, (row+0.5)*0.8, math.radians(settings['start_heading_deg']))
            if self.map.start_pose is not None and any(abs(a-b) > 0.001 for a, b in zip(self.map.start_pose, start)):
                raise ValueError('Robot start pose differs from training map frame')
            self.map.start_pose = start
            self.map.add_object('start', *start[:2], 'start', label='старт / пункт сбора')
            self.odom.reset(start[0]*1000, start[1]*1000, start[2])
            self.nav = Navigator(self.motor, self.lidar, self.odom, self.map, settings, self.camera)
            self.observer = SemanticObserver(settings['camera_yaws_deg'], settings['vision_model'])
            self.planner = LLMPlanner()
            deadline = time.monotonic()+10
            while len(self.camera.get_frames()) != 3 and time.monotonic() < deadline:
                time.sleep(0.1)
            self.nav.sense()
            if settings.get('operator_cleared_start_gap'):
                # Only the three specifically identified unobserved start cells.
                # A measured occupied cell is never cleared by this declaration.
                cleared=[]
                for dx,dy in [(-.175,.075),(-.125,.075),(-.175,.125)]:
                    x=start[0]+dx*math.cos(start[2])-dy*math.sin(start[2])
                    y=start[1]+dx*math.sin(start[2])+dy*math.cos(start[2])
                    cell=self.map.grid.index(x,y)
                    if self.map.grid.inside(cell) and not self.map.grid.observed[cell]:
                        self.map.grid.observed[cell]=True
                        self.map.grid.log_odds[cell]=-5
                        cleared.append([x,y])
                self.map.objects['start']['operator_clearance_evidence']={'source':'explicit operator check of left-rear start gap','cells':cleared,'time':time.time()}
                self.event('operator_start_clearance',cells=cleared,source='operator, not lidar')
            # Arming holds the serial bus during model/mode checks. Restart the
            # stationary encoder baseline afterwards rather than reporting a
            # false stale-odometry fault from initialization.
            self.odom.shutdown()
            self.resources.remove(self.odom)
            self.ensure_power_ready(settings.get('operator_accepts_power_faults', False))
            self.motor.arm()
            self.odom = Odometry(self.motor, LEFT_MOTORS, RIGHT_MOTORS,
                                 settings['wheel_diameter_mm'], settings['wheelbase_mm'], angular_scale=settings.get('odometry_angular_scale',1.0))
            self.resources.append(self.odom)
            self.odom.reset(start[0]*1000, start[1]*1000, start[2])
            self.nav.odometry = self.odom
            self.guard = threading.Thread(target=self._guard_loop, daemon=True)
            self.guard.start()
            self.power_guard=threading.Thread(target=self._power_guard_loop,daemon=True)
            self.power_guard.start()
        except BaseException:
            self.shutdown()
            raise

    @staticmethod
    def ensure_power_ready(operator_accepts_power_faults=False):
        status=read_status()
        if status['current_fault'] and not operator_accepts_power_faults:
            raise RuntimeError(f"Current Raspberry Pi power/throttling fault: {status['raw']}")
        return status

    def _power_guard_loop(self):
        while not self.guard_stop.wait(0.5):
            try:
                status = self.ensure_power_ready(getattr(self, 'settings', {}).get('operator_accepts_power_faults',False))
                if hasattr(self, 'report'):
                    self.report['last_power_status'] = status
            except Exception as exc:
                if not self.guard_stop.is_set():
                    self.motor.abort(f'Power monitor: {exc}')
                return

    def _guard_loop(self):
        while not self.guard_stop.wait(0.1):
            try:
                if self.mission_deadline and time.monotonic() >= self.mission_deadline:
                    raise RuntimeError('Five-minute mission deadline exceeded')
                if not self.lidar.get_fresh_scan(timeout=0):
                    raise RuntimeError('Lidar became stale')
                if len(self.camera.get_frames()) != 3:
                    raise RuntimeError('Camera became stale')
                self.odom.get_pose()
            except Exception as exc:
                self.motor.abort(str(exc))
                return

    def confirm_victim_stop(self, qr, result):
        self.motor.stop_all()
        # Estimate the QR bearing from its camera; exact camera calibration is
        # required before a competition run. Lidar provides measured range.
        scan, initial = self.nav.sense()
        yaw = self.settings['camera_yaws_deg'][qr['camera']]
        distances = [d/1000 for q, a, d in scan if q > 0 and abs((self.settings['lidar_angle_sign']*(a-self.settings['lidar_front_deg'])-yaw+180)%360-180) < 15]
        if not distances or min(distances) > 0.8:
            raise RuntimeError('Victim QR is not confirmed nearby by range measurements')
        bearing = initial[2]+math.radians(yaw)
        victim_x, victim_y = initial[0]+min(distances)*math.cos(bearing), initial[1]+min(distances)*math.sin(bearing)
        if not 0 <= victim_x < 4 or not 0 <= victim_y < 4:
            raise RuntimeError('QR range falls outside arena coordinates')
        # Choose the closest accessible cell center rather than the nearest
        # point to the large landmark, which may occupy multiple cells.
        choices = []
        for row in range(5):
            for col in range(5):
                point = (col+0.5)*0.8, (row+0.5)*0.8
                if self.map.grid.route(initial[:2], point, self.nav.radius):
                    choices.append((math.hypot(point[0]-victim_x,point[1]-victim_y), row, col, point))
        if not choices:
            raise RuntimeError('No accessible cell for the victim stop')
        _, row, col, point = min(choices)
        self.nav.navigate(point)
        self.motor.stop_all()
        _, initial = self.nav.sense()
        started = time.monotonic()
        while time.monotonic()-started < 5:
            _, pose = self.nav.sense()
            if math.hypot(pose[0]-initial[0],pose[1]-initial[1]) > 0.02:
                raise RuntimeError('Robot moved during the required five-second stop')
            if any(abs(self.motor.read_speed(i)) > 5 for i in (2,7,8,9)):
                raise RuntimeError('Wheel motion detected during victim stop')
            time.sleep(0.1)
        completed = time.monotonic()
        fresh, result = self.read_fresh_victim_qr(qr['data'], completed)
        self.report['victim'] = {'raw_qr': fresh['data'], **result}
        self.event('qr_confirmed_after_stop', raw=fresh['data'], assessment=result, camera=fresh['camera'])
        self.event('victim_stop_confirmed', duration_s=completed-started,
                   cell=[col,row], coordinate_order='x,y', pose=initial, victim_position_estimate=[victim_x,victim_y])

    def read_fresh_victim_qr(self, expected_text, not_before, timeout=10):
        self.motor.stop_all()
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.nav.sense()
            qr = self.camera.get_qr_code(timeout=0.1)
            if not qr or qr.get('timestamp', -1) < not_before or qr['data'] != expected_text:
                continue
            self.event('qr_detected',raw=qr['data'],camera=qr['camera'])
            assessment = self.planner.interpret_qr(qr['data'])
            if not assessment.get('is_victim'):
                raise RuntimeError('Fresh QR does not identify a victim')
            return qr, assessment
        raise RuntimeError('Victim QR was not freshly read after the five-second stop')

    def event(self, kind, **data):
        if kind in ('qr_detected','qr_read','qr_confirmed_after_stop'):
            print('QR: '+json.dumps({'event':kind,'camera':data.get('camera'),
                'raw_text':data.get('raw'),'victim_status':data.get('assessment',{}).get('status')},
                ensure_ascii=False),flush=True)
        self.report['events'].append({'time': time.time(), 'kind': kind, **data})
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.report_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.report, ensure_ascii=False, indent=2), encoding='utf-8')
        os.replace(temporary, self.report_path)
        from src.control.evidence_report import write_report
        write_report(self.report, self.map, self.report_path.with_suffix('.html'))
        print(f'[{kind}] {json.dumps(data, ensure_ascii=False)}', flush=True)

    def observe_station(self):
        self.motor.stop_all()
        scan, pose = self.nav.sense()
        observations = self.observer.observe(self.camera.get_frames(), scan, pose, self.map,
                                             self.settings['lidar_front_deg'], self.settings['lidar_angle_sign'])
        self.event('observations', data=observations)

    def train(self, map_path, duration=300):
        # Map accessible free-space frontiers; do not route through unknown cells.
        import numpy as np
        deadline = time.monotonic()+duration
        visited = set()
        self.map.training_complete = False
        self.mission_deadline = deadline
        self.event('training_started', deadline_seconds=duration)
        explore_deadline = deadline-60
        while time.monotonic() < explore_deadline:
            settle = time.monotonic()+.5
            while time.monotonic()<settle:
                self.nav.sense(); time.sleep(.1)
            self.observe_station()
            self.map.save(map_path)
            _, pose = self.nav.sense()
            grid = self.map.grid
            blocked = grid.blocked(self.nav.radius)
            candidates = []
            for row, col in np.argwhere(~blocked):
                if row % 8 or col % 8 or (int(row), int(col)) in visited:
                    continue
                # Favor points near unexplored space that can already be reached safely.
                unknown = ~grid.observed[max(0,row-8):row+9, max(0,col-8):col+9]
                point = grid.point((row, col))
                # Visit observed reachable stations even if the initial lidar
                # already mapped the surrounding floor: cameras still need views.
                if any(v < .35 or v > grid.size-.35 for v in point):
                    continue
                distance = math.hypot(point[0]-pose[0],point[1]-pose[1])
                if distance < .15:
                    continue
                route = grid.route(pose[:2], point, self.nav.radius)
                if route:
                    heading = math.atan2(point[1]-pose[1],point[0]-pose[0])
                    turn = abs(math.atan2(math.sin(heading-pose[2]),math.cos(heading-pose[2])))
                    candidates.append((len(route)+30*turn, int(row), int(col), point))
            if not candidates:
                self.motor.stop_all()
                if visited and any(o.get('type')!='start' for o in self.map.objects.values()):
                    self.event('training_frontiers_exhausted')
                    break
                self.event('training_waiting_for_clearance', source='fresh lidar; moving tanker may block routes')
                until=min(explore_deadline,time.monotonic()+3)
                while time.monotonic()<until:
                    self.nav.sense();time.sleep(.1)
                continue
            _, row, col, point = min(candidates)
            visited.add((row, col))
            self.event('training_waypoint',goal=list(point),pose=list(pose))
            self.nav.navigate(point, timeout=min(60, max(1, explore_deadline-time.monotonic())))
        # Return is part of training; incomplete return never marks map ready.
        self.nav.navigate(tuple(self.map.start_pose[:2]))
        self.nav.rotate_to(self.map.start_pose[2])
        self.map.training_complete = any(obj.get('type') != 'start' for obj in self.map.objects.values())
        self.map.save(map_path)
        if not self.map.training_complete:
            raise RuntimeError('Survey returned to start but no landmark was recorded; map is not ready')
        self.event('training_completed', objects=len(self.map.objects), observed_fraction=float(self.map.grid.observed.mean()))

    def find_victim_qr(self, target, timeout=90):
        self.motor.stop_all()
        deadline = time.monotonic()+timeout
        _, pose = self.nav.sense()
        route = self.map.grid.approach(pose[:2], (target['x'], target['y']), self.nav.radius)
        if not route:
            raise RuntimeError('No safe observation position near target landmark')
        self.nav.navigate(route[-1])
        _, pose = self.nav.sense()
        self.nav.rotate_to(math.atan2(target['y']-pose[1], target['x']-pose[0]))
        searched = []
        while time.monotonic() < deadline:
            self.nav.sense()
            qr = self.camera.get_qr_code(timeout=0.1)
            if qr:
                # Wheels remain stopped during potentially slow LLM inference.
                self.motor.stop_all()
                self.event('qr_detected',raw=qr['data'],camera=qr['camera'])
                result = self.planner.interpret_qr(qr['data'])
                self.event('qr_read', raw=qr['data'], assessment=result, camera=qr['camera'])
                if result['is_victim']:
                    self.confirm_victim_stop(qr, result)
                    return result
            _, pose = self.nav.sense()
            candidates = []
            for angle in range(0, 360, 30):
                a = math.radians(angle)
                point = target['x']+0.35*math.cos(a), target['y']+0.35*math.sin(a)
                if point in searched:
                    continue
                route = self.map.grid.route(pose[:2], point, self.nav.radius)
                if route:
                    candidates.append((len(route), point))
            if not candidates:
                break
            _, point = min(candidates)
            searched.append(point)
            self.nav.navigate(point, timeout=min(30, max(1, deadline-time.monotonic())))
            self.nav.rotate_to(math.atan2(target['y']-point[1], target['x']-point[0]))
            # Allow fresh QR frames after each changed viewpoint.
            until = min(deadline, time.monotonic()+2)
            while time.monotonic() < until:
                qr = self.camera.get_qr_code(timeout=0.1)
                if qr:
                    self.event('qr_detected',raw=qr['data'],camera=qr['camera'])
                    result = self.planner.interpret_qr(qr['data'])
                    self.event('qr_read', raw=qr['data'], assessment=result, camera=qr['camera'])
                    if result['is_victim']:
                        self.confirm_victim_stop(qr, result)
                        return result
        raise RuntimeError('No verified victim QR found near the selected landmark')

    def confirm_landmark_arrival(self, object_id, target):
        # A saved map label alone is not evidence that the robot found the object.
        _, pose = self.nav.sense()
        route = self.map.grid.approach(pose[:2], (target['x'],target['y']),self.nav.radius)
        if not route:
            raise RuntimeError('No observed route to the selected landmark')
        self.nav.navigate(route[-1])
        _, pose = self.nav.sense()
        self.nav.rotate_to(math.atan2(target['y']-pose[1],target['x']-pose[0]))
        self.motor.stop_all()
        deadline = time.monotonic()+8
        expected = target.get('template_identity')
        if not expected:
            raise RuntimeError('Selected landmark needs a reviewed visual template identity')
        import cv2
        while time.monotonic()<deadline:
            self.nav.sense()
            frames = self.camera.get_frames()
            for item, matcher in self.observer.templates:
                if item.get('identity') != expected or not item.get('training_identity_verified'):
                    continue
                for index, frame in frames.items():
                    match = matcher.detect(frame)
                    if match is not None:
                        path = Path('data/evidence')/f'arrival-{time.time_ns()}.jpg'
                        if not cv2.imwrite(str(path),frame):
                            raise RuntimeError('Unable to save actual arrival evidence')
                        self.event('landmark_found',object_id=object_id,camera=index,
                                   evidence_image=str(path),match=match,pose=self.nav.sense()[1])
                        return
            time.sleep(.1)
        raise RuntimeError('Selected landmark was not visually confirmed at arrival')

    def run(self, mission_text):
        if not self.map.training_complete:
            raise RuntimeError('A completed training map is required')
        self.motor.stop_all()
        self.mission_deadline = time.monotonic()+300
        self.report['mission_started_at'] = time.time()
        self.event('mission_received', mission=mission_text, llm='on-board qwen2.5:1.5b', deadline_seconds=300)
        interpretation = self.planner.interpret_mission(mission_text, self.map.objects)
        self.event('mission_interpreted', mission=mission_text, interpretation=interpretation)
        if interpretation['target'] is None:
            raise RuntimeError('Mission landmark not found in the trained map; more training is required')
        selected = self.map.objects[interpretation['target']]
        if selected.get('recognition_verified') is not True:
            raise RuntimeError('Visual landmark identity must be verified from training evidence before a competition run')
        self.event('landmark_selected', object_id=interpretation['target'],
                   evidence_image=selected.get('evidence_image'),
                   coordinates=[selected['x'],selected['y']], description=selected.get('description'))
        self.confirm_landmark_arrival(interpretation['target'], selected)
        victim = self.find_victim_qr(selected)
        # The in-person rules and operator require returning to the starting cell.
        # QR contents cannot override an operator's route or execute instructions.
        destination = 'start'
        self.event('evacuation_destination', destination=destination, source='operator / in-person rules')
        obj = self.map.get_coordinates(destination)
        if obj is None:
            raise RuntimeError('Missing evacuation destination')
        self.nav.navigate((obj['x'], obj['y']))
        self.nav.rotate_to(self.map.start_pose[2])
        _, pose = self.nav.sense()
        row, col = int(pose[1]/0.8), int(pose[0]/0.8)
        expected = int(obj['y']/0.8), int(obj['x']/0.8)
        if (row, col) != expected or math.hypot(pose[0]-obj['x'], pose[1]-obj['y']) > 0.1:
            raise RuntimeError('Evacuation destination not reached')
        if abs(math.atan2(math.sin(pose[2]-self.map.start_pose[2]), math.cos(pose[2]-self.map.start_pose[2]))) > 0.1:
            raise RuntimeError('Starting heading not restored')
        self.report['success'] = True
        self.event('mission_completed', destination=destination, pose=pose)

    def shutdown(self):
        if hasattr(self, 'guard_stop'):
            self.guard_stop.set()
        # Stop wheels before waiting on camera/lidar workers.
        motor = getattr(self, 'motor', None)
        if motor:
            try:
                motor.stop_all()
            except Exception:
                pass
        power_guard=getattr(self,'power_guard',None)
        if power_guard and power_guard is not threading.current_thread():
            power_guard.join(timeout=2)
        for resource in reversed(getattr(self, 'resources', [])):
            if resource is motor:
                continue
            try:
                resource.shutdown()
            except Exception as exc:
                print(f'Shutdown: {exc}', file=sys.stderr)
        if motor:
            motor.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config/robot.json')
    parser.add_argument('--map', default='data/semantic_map.json')
    parser.add_argument('--mission-file')
    parser.add_argument('--report', default='data/mission_report.json')
    parser.add_argument('--train', action='store_true')
    parser.add_argument('--duration', type=float, default=300)
    parser.add_argument('--operator-cleared-start-gap', action='store_true', help='Operator checked the three specified left-rear blind cells at start')
    parser.add_argument('--rehearsal', action='store_true', help='Real field trial with unverified global localization explicitly recorded')
    parser.add_argument('--operator-accepts-power-faults', action='store_true', help='Record voltage faults without stopping; rehearsal only')
    parser.add_argument('--drive', action='store_true', help='Explicitly enable physical wheel motion')
    args = parser.parse_args()
    settings = load_settings(args.config)
    if args.operator_accepts_power_faults and not args.rehearsal:
        parser.error('Power risk acceptance is only available for a rehearsal')
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 300:
        parser.error('Duration must be between 1 and 300 seconds')
    settings['operator_cleared_start_gap'] = args.operator_cleared_start_gap
    settings['rehearsal'] = args.rehearsal
    settings['operator_accepts_power_faults'] = args.operator_accepts_power_faults
    if args.rehearsal:
        settings['scan_localization'] = True
        settings['drive_speed'] = min(settings['drive_speed'],180)
        settings['turn_speed'] = min(settings['turn_speed'],100)
    semantic_map = SemanticMap()
    if not args.train:
        semantic_map.load(args.map)
    mission = Path(args.mission_file).read_text(encoding='utf-8') if args.mission_file else None
    if not args.train and not mission:
        parser.error('--mission-file is required; no hardcoded missions')
    if not args.drive:
        if args.train:
            parser.error('Training requires --drive and measured calibration')
        print(json.dumps(LLMPlanner().interpret_mission(mission, semantic_map.objects), ensure_ascii=False, indent=2))
        return
    app = None
    try:
        app = MissionApp(settings, semantic_map, args.report)
        if args.train:
            app.train(args.map, args.duration)
        else:
            app.run(mission)
    except BaseException as exc:
        if app:
            app.map.save(Path(args.report).with_suffix('.map.json'))
            app.event('failed', error=str(exc), pose=list(app.map.robot_pose))
        raise
    finally:
        if app:
            app.shutdown()


if __name__ == '__main__':
    main()
