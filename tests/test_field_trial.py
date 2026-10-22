import math
import unittest
from unittest.mock import patch
import numpy as np
from src.control.scan_localization import ScanLocalization, relative_pose, compose_pose
from scripts.mission import validate_drive, MissionApp


class FieldTrialTests(unittest.TestCase):
    def test_unverified_localization_only_explicit_rehearsal_with_lidar(self):
        settings={'geometry_verified':True,'camera_geometry_verified':True,'lidar_geometry_verified':True,
                  'localization_verified':False,'start_cell':[0,0],'start_heading_deg':0,
                  'camera_yaws_deg':[0,-90,90],'drive_speed':120,'turn_speed':100}
        with self.assertRaises(ValueError):validate_drive(settings)
        settings['rehearsal']=True
        with self.assertRaises(ValueError):validate_drive(settings)
        settings['scan_localization']=True
        validate_drive(settings)
        self.assertFalse(settings['localization_verified'])

    def test_operator_power_acceptance_is_explicit(self):
        with patch('scripts.mission.read_status',return_value={'current_fault':True,'raw':'throttled=0x50005'}):
            self.assertTrue(MissionApp.ensure_power_ready(True)['current_fault'])
            with self.assertRaises(RuntimeError):MissionApp.ensure_power_ready()

    def test_local_frame_composition_with_rotation(self):
        start=(1.,2.,math.pi/2);end=(1.,2.2,math.pi/2+.1)
        delta=relative_pose(start,end)
        self.assertAlmostEqual(delta[0],.2)
        for a,b in zip(compose_pose(start,delta),end):self.assertAlmostEqual(a,b)

    def test_lidar_encoder_disagreement_rejected(self):
        loc=ScanLocalization({'lidar_front_deg':180,'lidar_angle_sign':-1})
        points=np.column_stack((np.linspace(-1,1,100),np.sin(np.linspace(-1,1,100))))
        with patch.object(loc,'points',return_value=points):
            loc.update([],1,(.4,.4,0))
            with patch('src.control.scan_localization.align_scans',return_value={'accepted':True,'translation_m':[0,0],'yaw_rad':0}):
                with self.assertRaisesRegex(RuntimeError,'disagree'):loc.update([],2,(.6,.4,0))

    def test_lidar_measurement_corrects_small_encoder_drift(self):
        loc=ScanLocalization({'lidar_front_deg':180,'lidar_angle_sign':-1})
        points=np.zeros((100,2))
        with patch.object(loc,'points',return_value=points):
            loc.update([],1,(.4,.4,0))
            with patch('src.control.scan_localization.align_scans',return_value={'accepted':True,'translation_m':[.09,0],'yaw_rad':0}):
                pose=loc.update([],2,(.5,.4,0))
                self.assertAlmostEqual(pose[0],.49)

    def test_ambiguous_localization_expires(self):
        loc=ScanLocalization({'lidar_front_deg':180,'lidar_angle_sign':-1})
        with patch.object(loc,'points',return_value=np.zeros((100,2))):
            loc.update([],1,(.4,.4,0));loc.last_accepted-=2
            with patch('src.control.scan_localization.align_scans',return_value={'accepted':False}):
                with self.assertRaisesRegex(RuntimeError,'ambiguous'):loc.update([],2,(.41,.4,0))

    def test_start_clearance_uses_actual_position_not_cell_center(self):
        from src.control.grid_map import GridMap
        g=GridMap();g.observed[:]=True;g.log_odds[:]=-5
        g.observed[7,3]=False
        start=(.399999,.399999);radius=.191421356
        self.assertTrue(g.blocked(radius)[g.index(*start)])
        self.assertTrue(g.is_position_clear(start,radius))
        self.assertTrue(g.route(start,(.625,.425),radius))
        self.assertFalse(g.is_position_clear((.375,.375),radius))

    def test_unknown_rectangles_and_arena_edges_still_block_actual_position(self):
        from src.control.grid_map import GridMap
        g=GridMap();g.observed[:]=True;g.log_odds[:]=-5
        g.observed[7,4]=False
        self.assertFalse(g.is_position_clear((.4,.4),.191))
        self.assertFalse(g.route((.4,.4),(.625,.425),.191))
        self.assertFalse(g.is_position_clear((.1,.4),.191))

    def test_encoder_transport_retry_requires_real_response(self):
        import threading
        from unittest.mock import Mock
        from dynamixel_sdk import COMM_SUCCESS, COMM_RX_TIMEOUT
        from src.hardware.motor_controller import MotorController
        m=MotorController.__new__(MotorController);m._port_lock=threading.RLock()
        m.portHandler=Mock();m.packetHandler=Mock();m._check=Mock(side_effect=RuntimeError('timeout'))
        m.packetHandler.read2ByteTxRx.side_effect=[(0,COMM_RX_TIMEOUT,0),(1234,COMM_SUCCESS,0)]
        self.assertEqual(m._read(2,36),1234)
        self.assertEqual(m.read_retry_count,1)
        m._check.assert_not_called()
        m.packetHandler.read2ByteTxRx.side_effect=[(0,COMM_RX_TIMEOUT,0),(0,COMM_RX_TIMEOUT,0)]
        with self.assertRaises(RuntimeError):m._read(2,36)

    def test_encoder_device_alarm_is_not_retried(self):
        import threading
        from unittest.mock import Mock
        from dynamixel_sdk import COMM_SUCCESS
        from src.hardware.motor_controller import MotorController
        m=MotorController.__new__(MotorController);m._port_lock=threading.RLock()
        m.portHandler=Mock();m.packetHandler=Mock();m._check=Mock(side_effect=RuntimeError('alarm'))
        m.packetHandler.read2ByteTxRx.return_value=(1234,COMM_SUCCESS,4)
        with self.assertRaises(RuntimeError):m._read(2,36)
        self.assertEqual(m.packetHandler.read2ByteTxRx.call_count,1)

    def test_missed_broadcast_is_repaired_and_confirmed_per_wheel(self):
        from unittest.mock import Mock
        from src.hardware.motor_controller import MotorController
        m=MotorController.__new__(MotorController);m._write=Mock()
        m._read=Mock(side_effect=[1,0,60,55])
        result=m.verify_command({9:60})
        m._write.assert_called_once_with(9,32,60)
        self.assertEqual(result[9]['goal_speed'],60)

    def test_disabled_wheel_is_not_rearmed_during_motion(self):
        from unittest.mock import Mock
        from src.hardware.motor_controller import MotorController
        m=MotorController.__new__(MotorController);m._write=Mock();m._read=Mock(return_value=0)
        with self.assertRaisesRegex(RuntimeError,'torque disabled'):m.verify_command({9:60})
        m._write.assert_not_called()

    def test_operator_coordinates_are_xy_and_not_claimed_as_detections(self):
        from scripts.rehearsal import seed_map
        layout={'landmarks':[{'label':'yellow damaged building','cells':[[1,2],[1,3]]}]}
        catalog=[{'identity':'yellow_damaged_building','description':'A yellow damaged building.'}]
        m=seed_map(layout,catalog);obj=m.objects['yellow_damaged_building']
        self.assertAlmostEqual(obj['x'],1.2);self.assertAlmostEqual(obj['y'],2.4)
        self.assertFalse(obj['recognition_verified'])
        self.assertEqual(obj['coordinate_source'],'operator layout')

    def test_shortcut_removes_grid_zigzag_only_in_observed_free_space(self):
        from src.control.grid_map import GridMap
        from src.control.semantic_map import SemanticMap
        from src.control.navigation import Navigator
        from unittest.mock import Mock
        m=SemanticMap();m.grid.observed[:]=True;m.grid.log_odds[:]=-5
        settings={'robot_width_m':.2,'robot_length_m':.2,'clearance_m':.05}
        nav=Navigator(Mock(),Mock(),Mock(),m,settings)
        route=[(.4,.4),(.8,.4),(.8,.8)]
        self.assertEqual(nav.shortcut_route(route,(.4,.4),(.8,.8)),[(.8,.8)])
        m.grid.observed[:]=False
        self.assertFalse(nav.line_clear((.4,.4),(.8,.8)))

    def test_outward_deadline_stops_without_latching_motor_fault(self):
        from unittest.mock import Mock
        from src.control.navigation import Navigator
        from src.control.semantic_map import SemanticMap
        motor=Mock();motor.fault=None
        nav=Navigator(motor,Mock(),Mock(),SemanticMap(),{'robot_width_m':.2,'robot_length_m':.2,'clearance_m':.05})
        nav.phase_deadline=0
        with self.assertRaisesRegex(RuntimeError,'reserve return'):nav.sense()
        motor.stop_all.assert_called_once();motor.abort.assert_not_called()
        self.assertIsNone(motor.fault)
