import json
import unittest
from pathlib import Path
from src.control.mission_objects import mentioned_objects,explicit_search_focus
class MissionExamplesTests(unittest.TestCase):
 def test_all_five_user_prompts_include_all_objects(self):
  cases=json.loads(Path('data/mission-examples.json').read_text(encoding='utf-8'))
  for case in cases:
   with self.subTest(prompt=case['text']):
    items=mentioned_objects(case['text'])
    self.assertEqual(explicit_search_focus(case['text'],items),case['expected_search_near'])
    self.assertEqual({item['id'] for item in items},set(case['expected_objects']))
    for item in items:self.assertIn(item['quote'],case['text'])

 def test_twenty_additional_prompts_name_all_objects(self):
  cases=json.loads(Path('data/mission-additional-holdout.json').read_text(encoding='utf-8'))
  for case in cases:
   with self.subTest(case=case['case']):
    self.assertEqual({i['id'] for i in mentioned_objects(case['text'])},set(case['expected_objects']))
