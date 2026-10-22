import unittest
from src.hardware.power_status import decode_status

class PowerStatusTests(unittest.TestCase):
    def test_history_does_not_indicate_current_fault(self):
        result=decode_status('throttled=0x50000')
        self.assertFalse(result['current_fault'])
        self.assertTrue(result['undervoltage_occurred'])
        self.assertTrue(result['throttling_occurred'])
    def test_current_undervoltage_is_fault(self):
        result=decode_status('throttled=0x50005')
        self.assertTrue(result['current_fault'])
        self.assertTrue(result['undervoltage_now'])
    def test_invalid_report_is_not_healthy(self):
        with self.assertRaises(ValueError):decode_status('invalid')
