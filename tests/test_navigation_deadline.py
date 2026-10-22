import unittest
from unittest.mock import Mock,patch
from src.control.navigation import Navigator
from scripts.rehearsal import seed_map,return_budget
class NavigationDeadlineTests(unittest.TestCase):
 def test_nested_rotation_uses_segment_remaining_time(self):
  nav=Navigator.__new__(Navigator);nav.motor=Mock()
  nav.sense=Mock(return_value=([],(0,0,1)))
  nav.rotate_to=Mock(side_effect=RuntimeError('Rotation timeout'))
  with patch('src.control.navigation.time.monotonic',side_effect=[10,10,11]):
   with self.assertRaises(RuntimeError):nav._follow_segment((1,0),timeout=3)
  self.assertEqual(nav.rotate_to.call_args.kwargs['timeout'],2)
  nav.motor.stop_all.assert_called()
 def test_cup_is_operator_defined_blue_building(self):
  m=seed_map({'landmarks':[{'label':'cup','cells':[[3,1]]}]},[])
  obj=next(iter(m.objects.values()))
  self.assertEqual(obj['category'],'blue_building')
  self.assertEqual(obj['type'],'building')
  self.assertFalse(obj['recognition_verified'])
 def test_return_budget_uses_safe_shortcut_path(self):
  app=Mock();app.nav.sense.return_value=([],(.4,.4,0))
  app.map.start_pose=(1.4,.4,0);app.nav.radius=.2
  app.map.grid.route.return_value=[(.4,.4),(.5,.4),(.5,.5),(1.4,.4)]
  app.nav.shortcut_route.return_value=[(1.4,.4)]
  self.assertAlmostEqual(return_budget(app),67.5)
