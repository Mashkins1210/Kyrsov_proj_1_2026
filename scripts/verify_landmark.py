"""Training-only confirmation of visual labels. Never invoke during an attempt."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from src.control.semantic_map import SemanticMap
from src.control.evidence_report import write_report

parser=argparse.ArgumentParser()
parser.add_argument('--map',required=True)
parser.add_argument('--name')
parser.add_argument('--description',help='Verified description of the actual image, in English for label matching')
args=parser.parse_args()
m=SemanticMap(); m.load(args.map)
if args.name:
    obj=m.objects[args.name]
    if not args.description or not obj.get('evidence_image') or not Path(obj['evidence_image']).is_file():
        parser.error('An actual saved observation image and verified description are required')
    obj['description']=args.description
    obj['recognition_verified']=True
    m.save(args.map)
report={'success':False,'events':[{'kind':'training_map_review','objects':m.objects}]}
write_report(report,m,'data/training_review.html')
print(m.objects)
