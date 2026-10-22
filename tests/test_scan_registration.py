import unittest,math
import numpy as np
from src.control.scan_registration import align_scans

class ScanRegistrationTests(unittest.TestCase):
    def scene(self):
        rng=np.random.default_rng(4)
        return rng.uniform([-1,-.7],[1,.7],(180,2))
    def test_known_small_translation_and_rotation(self):
        target=self.scene();angle=.025;rot=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]);shift=np.array([.045,-.012])
        source=(target-shift)@rot
        result=align_scans(source,target,initial=(.04,0,0))
        self.assertTrue(result['accepted']);np.testing.assert_allclose(result['translation_m'],shift,atol=.003);self.assertAlmostEqual(result['yaw_rad'],angle,delta=.003)
    def test_single_wall_is_ambiguous(self):
        wall=np.column_stack([np.linspace(-1,1,100),np.ones(100)])
        self.assertFalse(align_scans(wall,wall)['accepted'])
    def test_unrelated_distant_scan_is_rejected(self):
        target=self.scene();self.assertFalse(align_scans(target+10,target)['accepted'])
    def test_nonfinite_scan_rejected(self):
        target=self.scene();target[0,0]=float('nan')
        with self.assertRaises(ValueError):align_scans(target,self.scene())
    def test_stationary_scan_with_noise_has_small_drift(self):
        target=self.scene();noise=np.random.default_rng(5).normal(0,.001,target.shape)
        result=align_scans(target+noise,target)
        self.assertTrue(result['accepted']);self.assertLess(np.linalg.norm(result['translation_m']),.003)
