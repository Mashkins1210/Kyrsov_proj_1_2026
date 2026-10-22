import unittest
from unittest.mock import Mock,patch
from scripts.rehearsal import execute
from src.control.semantic_map import SemanticMap
class DynamicSearchTests(unittest.TestCase):
 def test_unknown_dynamic_target_is_discovered_during_search(self):
  app=Mock();app.map=SemanticMap();app.mission_deadline=500
  def observe():app.map.add_object('tree',1,1,'tree',category='fallen_tree',position_current=True,template_identity='fallen_tree',description='fallen tree')
  app.observe_station.side_effect=observe
  result=dict(target=None,requirements={'kind':'tree'},search_near=['fallen_tree'],relations=[])
  with patch('scripts.rehearsal.time.monotonic',return_value=10),patch('scripts.rehearsal.return_budget',return_value=60),patch('scripts.rehearsal.detect_target',return_value=True):
   execute(app,'mission',result)
  app.observe_station.assert_called_once()
  app.confirm_landmark_arrival.assert_called_once()
  app.find_victim_qr.assert_called_once()
  self.assertTrue(any(c.args[0]=='landmark_search_started' for c in app.event.call_args_list))
