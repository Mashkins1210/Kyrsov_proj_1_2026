import copy
import unittest
from src.control.mission_objects import STATIC,DYNAMIC,LABELS,extract_mission
class FakePlanner:
 def __init__(self,result):self.result=result
 def _ask(self,*args,**kwargs):return copy.deepcopy(self.result)
class MissionObjectsTests(unittest.TestCase):
 def test_all_regulation_categories(self):
  self.assertEqual(len(STATIC),5);self.assertEqual(len(DYNAMIC),8)
  self.assertFalse(STATIC & DYNAMIC)
  self.assertEqual(STATIC | DYNAMIC,set(LABELS))
  for key,label in LABELS.items():
   prompt=label
   landmark=key if key!='victim' else 'blue_building'
   if key=='victim':prompt+=' '+LABELS[landmark]
   objects=[{'id':key,'quote':label}]
   if landmark!=key:objects.append({'id':landmark,'quote':LABELS[landmark]})
   result=extract_mission(FakePlanner(dict(objects=objects,relations=[],key_landmarks=[landmark],search_near=[landmark])),prompt)
   item=result['objects'][0]
   self.assertEqual(item['requires_fresh_detection'],key in DYNAMIC)
 def test_reject_invented_object_quote(self):
  result=dict(objects=[dict(id='fallen_tree',quote='дерево')],relations=[],key_landmarks=['fallen_tree'],search_near=['fallen_tree'])
  with self.assertRaises(ValueError):extract_mission(FakePlanner(result),'Около здания')
 def test_reject_relationship_to_absent_object(self):
  result=dict(objects=[dict(id='fallen_tree',quote='дерево')],relations=[dict(subject='fallen_tree',object='blue_building',relation='fallen_on',quote='дерево')],key_landmarks=['fallen_tree'],search_near=['fallen_tree'])
  with self.assertRaises(ValueError):extract_mission(FakePlanner(result),'дерево')

 def test_reject_missing_named_objects(self):
  result=dict(objects=[dict(id='fallen_tree',quote='дерево')],relations=[],key_landmarks=['fallen_tree'],search_near=['fallen_tree'])
  with self.assertRaises(ValueError):extract_mission(FakePlanner(result),'Пострадавший около дерева. Рядом дерево и голубой дом.')
 def test_reject_wrong_fallen_relationship(self):
  prompt='Пострадавший около дерева, свалившимся на голубой дом'
  result=dict(objects=[dict(id='victim',quote='Пострадавший'),dict(id='fallen_tree',quote='дерева'),dict(id='blue_building',quote='голубой дом')],relations=[dict(subject='fallen_tree',object='blue_building',relation='near',quote=prompt)],key_landmarks=['fallen_tree','blue_building'],search_near=['fallen_tree'])
  with self.assertRaises(ValueError):extract_mission(FakePlanner(result),prompt)
