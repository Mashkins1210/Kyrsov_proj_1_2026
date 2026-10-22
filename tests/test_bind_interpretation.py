import unittest
from src.control.mission_objects import bind_interpretation
class BindInterpretationTests(unittest.TestCase):
 def extracted(self,focus):return dict(search_near=[focus],relations=[],search_constraints=[])
 def test_known_static_category(self):
  r=bind_interpretation(self.extracted('blue_building'),{'house':dict(category='blue_building',description='blue house')})
  self.assertEqual(r['target'],'house');self.assertFalse(r['spatial_relationships_verified'])
 def test_stale_dynamic_cannot_be_selected(self):
  r=bind_interpretation(self.extracted('fallen_tree'),{'tree':dict(category='fallen_tree',position_current=False)})
  self.assertIsNone(r['target'])
 def test_fresh_dynamic_category_can_be_selected(self):
  r=bind_interpretation(self.extracted('fallen_tree'),{'tree':dict(category='fallen_tree',position_current=True,description='tree')})
  self.assertEqual(r['target'],'tree')
 def test_ambiguous_instances_rejected(self):
  with self.assertRaises(ValueError):bind_interpretation(self.extracted('bridge'),{'a':dict(category='bridge'),'b':dict(category='bridge')})
