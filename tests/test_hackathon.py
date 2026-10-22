import math
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from src.control.robot_controller import RobotController
from src.perception.obstacle_detector import ObstacleDetector
from src.hardware.motor_controller import MotorController
from src.hardware.lidar_controller import LidarController
from src.control.odometry import Odometry
from src.control.llm_planner import LLMPlanner
from src.control.grid_map import GridMap
from src.control.semantic_map import SemanticMap
from src.control.evidence_report import write_report
from scripts.mission import validate_drive
from src.control.navigation import Navigator
from src.perception.camera_controller import CameraController
from queue import Queue


class RegressionTests(unittest.TestCase):
    def test_missing_scan_stops_in_every_translating_state(self):
        motor = Mock()
        controller = RobotController(motor, ObstacleDetector(), Mock(wall_side='left'))
        for state in ('handle_moving_state', 'handle_wall_following_state', 'handle_searching_wall_state'):
            motor.reset_mock()
            getattr(controller, state)(None)
            motor.stop_all.assert_called_once()
            motor.move_forward.assert_not_called()

    def test_cooldown_still_checks_front_obstacle(self):
        motor, state = Mock(), Mock(wall_side='left')
        controller = RobotController(motor, ObstacleDetector(), state)
        controller.wall_following_start_time = time.time()
        controller.handle_wall_following_state([(15, 180, 100)])
        state.start_turning.assert_called_once_with('left')
        motor.move_forward.assert_not_called()

    def test_wall_sides_are_physical(self):
        detector = ObstacleDetector()
        self.assertEqual(detector.detect_wall_side([(15, 90, 300)]*12)[0], 'left')
        self.assertEqual(detector.detect_wall_side([(15, 270, 300)]*12)[0], 'right')

    def test_motor_direction_encoding(self):
        self.assertEqual(MotorController.encode_velocity(9, 180), 180)
        self.assertEqual(MotorController.encode_velocity(2, 180), 180)
        self.assertEqual(MotorController.encode_velocity(8, 180), 1204)
        self.assertEqual(MotorController.encode_velocity(7, -180), 180)
        self.assertEqual(MotorController.encode_velocity(8, 9999), 2047)
        self.assertEqual(MotorController.encode_velocity(9, 9999), 1023)
        for v in (float('nan'), float('inf'), '3'):
            with self.assertRaises(ValueError):
                MotorController.encode_velocity(9, v)

    def test_unarmed_motion_rejected(self):
        m = MotorController.__new__(MotorController)
        m._port_lock = threading.RLock(); m.armed = False
        with self.assertRaises(RuntimeError):
            m.set_velocities_bulk({2: 100, 7: 100, 8: 100, 9: 100})
        m.set_velocities_bulk({2: 0, 7: 0, 8: 0, 9: 0})
        with self.assertRaises(ValueError):
            m.set_velocities_bulk({2: 10})

    def test_arm_rejects_joint_mode_without_writes(self):
        m = MotorController.__new__(MotorController)
        m._port_lock = threading.RLock(); m.fault = None
        m.inspect = lambda: {2: {'model': 320, 'cw_limit': 0, 'ccw_limit': 4095, 'torque_control': 0}}
        m._write = Mock()
        with self.assertRaises(RuntimeError):
            m.arm()
        m._write.assert_not_called()

    def test_arc_left_slows_left_side(self):
        m = MotorController.__new__(MotorController); m.set_velocities_bulk = Mock()
        m.move_forward_with_arc('left', 180, 30)
        self.assertEqual(m.set_velocities_bulk.call_args.args[0], {2: 150, 7: 210, 8: 210, 9: 150})

    def test_lidar_never_returns_stale_scan(self):
        lidar = LidarController.__new__(LidarController)
        lidar.scan_lock = threading.Condition(); lidar.stop_event = threading.Event()
        lidar.last_scan = [(15, 180, 500)]; lidar.last_scan_at = time.monotonic()-2
        lidar.max_age = 0.5
        self.assertIsNone(lidar.get_fresh_scan(timeout=0))
        lidar.last_scan_at = time.monotonic()
        self.assertEqual(lidar.get_fresh_scan(timeout=0), lidar.last_scan)

    def test_encoder_wrap_in_both_directions(self):
        odom = Odometry.__new__(Odometry); odom.ticks_per_rev = 4096
        self.assertEqual(odom._avg_delta({9: 10}, {9: 4090}, [9]), 16)
        self.assertEqual(odom._avg_delta({9: 4090}, {9: 10}, [9]), -16)

    def test_encoder_failure_not_treated_as_zero_motion(self):
        odom = Odometry.__new__(Odometry); odom._lock = threading.Lock()
        odom.fault = 'encoder lost'; odom.last_update = time.monotonic()
        with self.assertRaises(RuntimeError): odom.get_pose()

    def test_unknown_space_is_not_a_route(self):
        grid = GridMap()
        self.assertIsNone(grid.route((0.4,0.4), (3.6,3.6), 0.19))

    def test_astar_avoids_obstacle_and_returns(self):
        grid = GridMap(); grid.observed[:] = True; grid.log_odds[:] = -5
        grid.log_odds[20:60, 38:42] = 5
        outward = grid.route((0.4,0.4), (3.6,3.6), 0.19)
        homeward = grid.route((3.6,3.6), (0.4,0.4), 0.19)
        self.assertTrue(outward); self.assertTrue(homeward)
        blocked = grid.blocked(0.19)
        self.assertTrue(all(not blocked[grid.index(*p)] for p in outward))

    def test_robot_size_blocks_narrow_gap(self):
        grid = GridMap(); grid.observed[:] = True; grid.log_odds[:] = -5
        grid.log_odds[:,35:45] = 5; grid.log_odds[38:42,35:45] = -5
        self.assertIsNone(grid.route((0.4,2), (3.6,2), 0.19))

    def test_semantic_map_roundtrip_and_cells(self):
        m = SemanticMap(); m.add_object('yellow', 1.2,2,'building')
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'map.json'; m.save(p)
            other = SemanticMap(); other.load(p)
            self.assertEqual(m.objects, other.objects)
        self.assertEqual(m.get_coordinates('cell_4_4')['x'], 3.6)
        self.assertIsNone(m.get_coordinates('cell_9_1'))

    def test_malformed_occupancy_rejected(self):
        with self.assertRaises(ValueError):
            GridMap.from_data({'resolution':0.05, 'size':4, 'observed':[], 'log_odds':[]})

    def test_default_settings_prevent_uncalibrated_drive(self):
        with self.assertRaises(ValueError): validate_drive({'geometry_verified': False})

    def test_report_escapes_mission_html(self):
        report = {'success':False,'events':[{'kind':'mission_interpreted','mission':'<script>alert(1)</script>','interpretation':{}}]}
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'report.html'; write_report(report, SemanticMap(), p)
            text=p.read_text(encoding='utf-8')
            self.assertNotIn('<script>',text); self.assertIn('&lt;script&gt;',text)

    def test_camera_qr_discards_stale_detection(self):
        camera = CameraController.__new__(CameraController)
        camera.max_age = 1; camera.qr_queue = Queue()
        camera.qr_queue.put({'timestamp':time.monotonic()-5,'data':'old'})
        self.assertIsNone(camera.get_qr_code(timeout=0))
        camera.qr_queue.put({'timestamp':time.monotonic(),'data':'new'})
        self.assertEqual(camera.get_qr_code(timeout=0)['data'],'new')

    def test_qr_decoder_reads_actual_generated_code(self):
        import cv2
        code = cv2.QRCodeEncoder_create().encode('Victim: leg injury; conscious')
        image = cv2.copyMakeBorder(code,4,4,4,4,cv2.BORDER_CONSTANT,value=255)
        image = cv2.resize(image,None,fx=10,fy=10,interpolation=cv2.INTER_NEAREST)
        text, _, _ = cv2.QRCodeDetector().detectAndDecode(image)
        self.assertEqual(text,'Victim: leg injury; conscious')

    def test_watchdog_stops_and_latches_fault(self):
        m = MotorController.__new__(MotorController)
        m._port_lock = threading.RLock(); m.armed = True; m.last_speed_command=100
        m._last_command_at=time.monotonic()-1; m.watchdog_timeout=0.5; m._stop_event=threading.Event()
        m._emergency_stop=Mock(side_effect=lambda: m._stop_event.set())
        m._watchdog()
        m._emergency_stop.assert_called_once()
        self.assertIn('watchdog',m.fault)

    def test_navigation_missing_lidar_stops(self):
        m,lidar,odom=Mock(fault=None),Mock(),Mock()
        lidar.get_fresh_scan.return_value=None
        nav=Navigator(m,lidar,odom,SemanticMap(),{'robot_width_m':0.2,'robot_length_m':0.2,'clearance_m':0.05})
        with self.assertRaises(RuntimeError): nav.sense()
        m.stop_all.assert_called_once(); odom.get_pose_meters.assert_not_called()

    def test_path_simplification_keeps_corners(self):
        self.assertEqual(Navigator.corners([(0,0),(0,1),(0,2),(1,2),(2,2)]),[(0,2),(2,2)])

    def test_mission_requires_qr_then_returns_to_start_heading(self):
        from scripts.mission import MissionApp
        app=MissionApp.__new__(MissionApp)
        app.map=SemanticMap(); app.map.training_complete=True; app.map.start_pose=(0.4,0.4,0)
        app.map.add_object('start',0.4,0.4,'start')
        app.map.add_object('yellow',1.2,1.2,'building',recognition_verified=True)
        app.motor=Mock(); app.planner=Mock(); app.nav=Mock(); app.event=Mock()
        app.planner.interpret_mission.return_value={'target':'yellow','destination':'start'}
        app.confirm_landmark_arrival=Mock()
        app.report={'success':False}; app.find_victim_qr=Mock(return_value={'is_victim':True})
        app.nav.sense.return_value=([],(0.4,0.4,0))
        app.run('Найди у желтого здания')
        app.find_victim_qr.assert_called_once_with(app.map.objects['yellow'])
        app.nav.navigate.assert_called_once_with((0.4,0.4))
        app.nav.rotate_to.assert_called_once_with(0)
        self.assertTrue(app.report['success'])

    def test_no_qr_cannot_produce_mission_success(self):
        from scripts.mission import MissionApp
        app=MissionApp.__new__(MissionApp)
        app.map=SemanticMap(); app.map.training_complete=True
        app.map.add_object('yellow',1.2,1.2,'building',recognition_verified=True)
        app.motor=Mock(); app.planner=Mock(); app.nav=Mock(); app.event=Mock()
        app.planner.interpret_mission.return_value={'target':'yellow','destination':'start'}
        app.confirm_landmark_arrival=Mock()
        app.report={'success':False}; app.find_victim_qr=Mock(side_effect=RuntimeError('QR not found'))
        with self.assertRaises(RuntimeError): app.run('Найди у желтого здания')
        self.assertFalse(app.report['success']); app.nav.navigate.assert_not_called()

    def test_five_minute_deadline_stops_independently_of_navigation(self):
        from scripts.mission import MissionApp
        app=MissionApp.__new__(MissionApp); app.guard_stop=threading.Event()
        app.mission_deadline=time.monotonic()-1; app.motor=Mock()
        app._guard_loop()
        app.motor.abort.assert_called_once_with('Five-minute mission deadline exceeded')


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.map = {'yellow_building': {'x':1.2,'y':1.2,'type':'building',
                                      'description':'A yellow damaged building with rubble.'}}
        self.valid = {'target':'yellow_building','destination':'start','reason':'описание совпадает',
                      'search_description':'пострадавший под обломками',
                      'requirements':{'color':'yellow','kind':'building','condition':'damaged'},
                      'target_evidence':'yellow damaged building'}
        self.planner=LLMPlanner()

    def interpret(self, answer):
        with patch.object(self.planner,'_ask',return_value=answer):
            return self.planner.interpret_mission('Найди пострадавшего у желтого разрушенного здания', self.map)

    def test_supported_target_is_accepted(self): self.assertEqual(self.interpret(self.valid)['target'],'yellow_building')
    def test_invented_target_is_rejected(self):
        with self.assertRaises(ValueError): self.interpret({**self.valid,'target':'invented'})
    def test_wrong_color_is_rejected(self):
        answer={**self.valid,'requirements':{**self.valid['requirements'],'color':'blue'}}
        with self.assertRaises(ValueError): self.interpret(answer)
    def test_wrong_object_type_is_rejected(self):
        answer={**self.valid,'requirements':{**self.valid['requirements'],'kind':'bridge'}}
        with self.assertRaises(ValueError): self.interpret(answer)
    def test_unsupported_observation_quote_is_rejected(self):
        with self.assertRaises(ValueError): self.interpret({**self.valid,'target_evidence':'invented evidence'})
    def test_out_of_field_destination_is_rejected(self):
        with self.assertRaises(ValueError): self.interpret({**self.valid,'destination':'cell_5_0'})
    def test_no_matching_landmark_is_not_success(self):
        with patch.object(self.planner,'_ask',return_value={**self.valid,'target':None}):
            self.assertIsNone(self.planner.get_plan('найди',self.map))
    def test_qr_state_requires_source_evidence(self):
        with patch.object(self.planner,'_ask',return_value={'is_victim':True,'status':'перелом','evidence':'перелом','destination':None}):
            with self.assertRaises(ValueError): self.planner.interpret_qr('У пострадавшего ожог')
    def test_qr_state_is_preserved(self):
        with patch.object(self.planner,'_ask',return_value={'is_victim':True,'status':'ожог','evidence':'ожог','destination':None}):
            self.assertEqual(self.planner.interpret_qr('У пострадавшего ожог')['status'],'ожог')
    def test_nonvictim_qr_does_not_count(self):
        with patch.object(self.planner,'_ask',return_value={'is_victim':False,'destination':None}):
            self.assertFalse(self.planner.interpret_qr('https://example.com')['is_victim'])
    def test_invented_qr_diagnosis_is_rejected_even_with_real_quote(self):
        with patch.object(self.planner,'_ask',return_value={'is_victim':True,'status':'перелом ноги','evidence':'ожог','destination':None}):
            with self.assertRaises(ValueError): self.planner.interpret_qr('У пострадавшего ожог')


if __name__ == '__main__': unittest.main()




