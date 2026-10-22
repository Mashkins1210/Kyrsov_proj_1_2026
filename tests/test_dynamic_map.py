import unittest
from src.control.semantic_map import SemanticMap
class DynamicMapTests(unittest.TestCase):
 def test_moved_tree_does_not_average_old_position(self):
  m=SemanticMap();m.add_object('tree',.4,.4,'tree',label='unknown tree',recognition_verified=True)
  m.begin_attempt();self.assertFalse(m.objects['tree']['recognition_verified'])
  self.assertFalse(m.objects['tree']['position_current'])
  name=m.observe('unknown tree','tree',3,2,.6,'tree','camera+lidar')
  self.assertEqual(name,'tree');self.assertEqual(m.objects[name]['x'],3)
  self.assertEqual(m.objects[name]['observations'],1)
 def test_static_coordinates_are_retained(self):
  m=SemanticMap();m.add_object('house',2,1,'building',category='blue_building')
  m.begin_attempt();self.assertEqual(m.objects['house']['mobility'],'static')
  self.assertNotIn('previous_position',m.objects['house'])
 def test_unknown_prior_not_confirmed(self):
  m=SemanticMap();m.add_object('unknown',1,1,recognition_verified=True)
  m.begin_attempt();self.assertFalse(m.objects['unknown']['recognition_verified'])
