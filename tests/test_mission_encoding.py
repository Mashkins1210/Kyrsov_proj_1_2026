import tempfile
import unittest
from pathlib import Path
from scripts.validate_mission_text import read_mission_text
class MissionEncodingTests(unittest.TestCase):
 def test_russian_text_is_preserved_exactly(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'text';text='Пострадавший возле голубого здания. Найди его.'
   p.write_bytes(text.encode('utf-8'));self.assertEqual(read_mission_text(p),text)
 def test_damaged_multibyte_input_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'text';p.write_bytes(b'\xd0\x9f\xd0\xd0\xbb')
   with self.assertRaises(ValueError):read_mission_text(p)
   self.assertEqual(p.read_bytes(),b'\xd0\x9f\xd0\xd0\xbb')
 def test_blank_input_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'text';p.write_text('  \n',encoding='utf-8')
   with self.assertRaises(ValueError):read_mission_text(p)
 def test_missing_file_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   with self.assertRaises(ValueError):read_mission_text(Path(d)/'missing')
