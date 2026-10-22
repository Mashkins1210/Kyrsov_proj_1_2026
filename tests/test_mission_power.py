import unittest
from unittest.mock import Mock,patch
from scripts.mission import MissionApp

class MissionPowerTests(unittest.TestCase):
    def test_historical_fault_does_not_block_ready(self):
        with patch('scripts.mission.read_status',return_value={'current_fault':False,'raw':'throttled=0x50000'}):
            self.assertFalse(MissionApp.ensure_power_ready()['current_fault'])
    def test_current_fault_blocks_ready(self):
        with patch('scripts.mission.read_status',return_value={'current_fault':True,'raw':'throttled=0x50005'}):
            with self.assertRaises(RuntimeError):MissionApp.ensure_power_ready()
    def test_monitor_fault_aborts_motors(self):
        app=MissionApp.__new__(MissionApp);app.guard_stop=Mock();app.guard_stop.wait.return_value=False;app.guard_stop.is_set.return_value=False;app.motor=Mock()
        with patch('scripts.mission.read_status',return_value={'current_fault':True,'raw':'throttled=0x50005'}):app._power_guard_loop()
        app.motor.abort.assert_called_once()
    def test_monitor_read_failure_aborts_motors(self):
        app=MissionApp.__new__(MissionApp);app.guard_stop=Mock();app.guard_stop.wait.return_value=False;app.guard_stop.is_set.return_value=False;app.motor=Mock()
        with patch('scripts.mission.read_status',side_effect=RuntimeError('read timeout')):app._power_guard_loop()
        app.motor.abort.assert_called_once()
    def test_monitor_history_allows_operation(self):
        app=MissionApp.__new__(MissionApp);app.guard_stop=Mock();app.guard_stop.wait.side_effect=[False,True];app.motor=Mock()
        with patch('scripts.mission.read_status',return_value={'current_fault':False,'raw':'throttled=0x50000'}):app._power_guard_loop()
        app.motor.abort.assert_not_called()
