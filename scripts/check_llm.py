"""LLM acceptance checks using labeled synthetic maps, never drive hardware."""
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.control.llm_planner import LLMPlanner
from src.control.semantic_map import SemanticMap
from src.control.evidence_report import write_report

planner = LLMPlanner()
observations = {
    'yellow_ruined_building': {'x':1.2,'y':2.0,'type':'building','label':'yellow building',
        'description':'A yellow damaged building with collapsed walls and rubble around it.'},
    'blue_building': {'x':2.8,'y':2.8,'type':'building','label':'blue building',
        'description':'A blue building with windows and intact walls.'},
    'bridge': {'x':2.0,'y':0.8,'type':'bridge','label':'bridge',
        'description':'A bridge with yellow railings over water.'},
    'start': {'x':0.4,'y':0.4,'type':'start','label':'старт','description':'Starting cell.'}}
mission = 'Пострадавший попал под обломки обрушивщегося желтого здания. необходимо найти его среди них.'
result = planner.interpret_mission(mission, observations)
assert result['target'] == 'yellow_ruined_building', result
assert result['destination'] == 'start', result
assert result['requirements'] == {'color':'yellow','kind':'building','condition':'damaged'}, result
qr = 'Пострадавший: Иван. Состояние: травма левой ноги, в сознании.'
assessment = planner.interpret_qr(qr)
assert assessment['is_victim'] and assessment['evidence'] in qr, assessment
semantic_map = SemanticMap(); semantic_map.objects = observations
report = {'success':False, 'victim':None, 'events':[
    {'kind':'test_notice', 'message':'СИНТЕТИЧЕСКАЯ КАРТА. Это проверка понимания LLM, а не доказательство нахождения объекта на полигоне.'},
    {'kind':'mission_interpreted','mission':mission,'interpretation':result},
    {'kind':'synthetic_qr_interpreted','raw':qr,'assessment':assessment}]}
write_report(report, semantic_map, 'data/llm_acceptance.html')
Path('data/llm_acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, indent=2))
