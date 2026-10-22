import unittest
from unittest.mock import Mock,patch
from scripts.mission import MissionApp

class FreshVictimQRTests(unittest.TestCase):
    def app(self, qr):
        app=MissionApp.__new__(MissionApp)
        app.event = Mock()
        app.motor=Mock();app.nav=Mock();app.camera=Mock();app.planner=Mock()
        app.camera.get_qr_code.return_value=qr
        app.planner.interpret_qr.return_value={'is_victim':True,'status':'в сознании'}
        return app
    def test_old_qr_cannot_confirm_stop(self):
        app=self.app({'data':'victim','timestamp':9,'camera':0})
        with patch('scripts.mission.time.monotonic',side_effect=[10,10,11]):
            with self.assertRaises(RuntimeError):app.read_fresh_victim_qr('victim',10,timeout=1)
        app.planner.interpret_qr.assert_not_called()
    def test_different_qr_cannot_confirm_victim(self):
        app=self.app({'data':'other','timestamp':10,'camera':0})
        with patch('scripts.mission.time.monotonic',side_effect=[10,10,11]):
            with self.assertRaises(RuntimeError):app.read_fresh_victim_qr('victim',10,timeout=1)
        app.planner.interpret_qr.assert_not_called()
    def test_fresh_matching_qr_is_interpreted(self):
        app=self.app({'data':'victim','timestamp':10.1,'camera':0})
        with patch('scripts.mission.time.monotonic',side_effect=[10,10]):
            qr,state=app.read_fresh_victim_qr('victim',10,timeout=1)
        self.assertTrue(state['is_victim']);app.motor.stop_all.assert_called_once()
        app.planner.interpret_qr.assert_called_once_with('victim')
    def test_fresh_nonvictim_is_rejected(self):
        app=self.app({'data':'victim','timestamp':10.1,'camera':0});app.planner.interpret_qr.return_value={'is_victim':False}
        with patch('scripts.mission.time.monotonic',side_effect=[10,10]):
            with self.assertRaises(RuntimeError):app.read_fresh_victim_qr('victim',10,timeout=1)
