"""Field session launcher. No wheels move for status/new-session commands."""
import argparse
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.control.semantic_map import SemanticMap


def save_session(path, session):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(session, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(temporary, path)


def readiness(map_path, catalog_path):
    problems = []
    details = {'map': str(map_path)}
    try:
        m = SemanticMap(); m.load(map_path)
        details['observed_fraction'] = float(m.grid.observed.mean())
        details['objects'] = m.objects
        if not m.training_complete:
            problems.append('Обучающий объезд не завершён с возвращением на старт')
        landmarks = [o for o in m.objects.values() if o.get('type') != 'start']
        if not landmarks:
            problems.append('В карте отсутствуют реальные ориентиры')
        for obj in landmarks:
            if not obj.get('recognition_verified') or not obj.get('template_identity'):
                problems.append('Ориентир не подтверждён сохранённым визуальным образцом')
    except (OSError, ValueError, KeyError) as exc:
        problems.append(f'Карта не готова: {exc}')
    try:
        catalog = json.loads(catalog_path.read_text(encoding='utf-8'))
        verified = {t['identity'] for t in catalog if t.get('training_identity_verified') and Path(t['image']).is_file()}
        details['verified_template_identities'] = sorted(verified)
        if not verified:
            problems.append('Нет проверенных визуальных образцов ориентиров')
        if 'm' in locals():
            for obj in m.objects.values():
                if obj.get('type') != 'start' and obj.get('template_identity') not in verified:
                    problems.append('Образец ориентира не проверен или отсутствует')
    except (OSError, ValueError, KeyError) as exc:
        problems.append(f'Каталог образцов не готов: {exc}')
    return {'ready_for_rehearsal': not problems, 'problems': problems, **details}


def main():
    os.chdir(ROOT)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status','new-session','train','run','intervention','stop'])
    parser.add_argument('--map', default='data/semantic_map.json')
    parser.add_argument('--config', default='config/robot.json')
    parser.add_argument('--mission-file')
    parser.add_argument('--session', default='data/field_session.json')
    parser.add_argument('--operator-cleared-start-gap',action='store_true')
    parser.add_argument('--start-confirmed', action='store_true', help='Operator confirms [0,0], configured heading, robot on arena')
    parser.add_argument('--operator-accepts-power-faults', action='store_true')
    args = parser.parse_args()
    session_path = Path(args.session)
    if args.action == 'stop':
        # Stop command producers before zeroing drives, so another loop cannot
        # overwrite the operator stop with its next positive speed packet.
        stopped=[]
        for entry in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                argv=entry.read_bytes().split(b'\0')
                if len(argv)<2 or b'python' not in argv[0]:
                    continue
                name=Path(argv[1].decode()).name
                if name not in ('mission.py','rehearsal.py','calibrate_drive.py','paired_scan_operator_power.py','long_scan_operator_power.py','drive_probe.py'):
                    continue
                pid=int(entry.parent.name)
                if pid!=os.getpid():
                    os.kill(pid,signal.SIGINT);stopped.append(pid)
            except (OSError,ValueError):
                pass
        deadline=time.monotonic()+8
        while any(Path(f'/proc/{pid}').exists() for pid in stopped) and time.monotonic()<deadline:
            time.sleep(.1)
        for pid in stopped:
            if Path(f'/proc/{pid}').exists():
                try:os.kill(pid,signal.SIGTERM)
                except ProcessLookupError:pass
        from src.hardware.motor_controller import MotorController
        from src.utils.config import DXL_IDS
        m = MotorController()
        errors = []
        try:
            for i in DXL_IDS:
                for register, value, size in [(32,0,2),(24,0,1)]:
                    try: m._write(i,register,value,size)
                    except Exception as exc: errors.append(str(exc))
        finally: m.shutdown()
        print(json.dumps({'stop_errors':errors},ensure_ascii=False))
        return 1 if errors else 0
    if args.action == 'new-session':
        session = {'created_at':time.time(),'attempts':[], 'max_attempts':2,'max_interventions_per_attempt':2}
        save_session(session_path,session)
        print('Создана новая пробная сессия: две попытки. Для официального испытания сброс сессии требует решения судей.')
        return 0
    if args.action == 'status':
        print(json.dumps(readiness(Path(args.map),Path('data/landmark_templates/catalog.json')),ensure_ascii=False,indent=2))
        return 0
    if args.action == 'intervention':
        if not args.start_confirmed:
            parser.error('Intervention requires physically returning the robot to the start and --start-confirmed')
        session=json.loads(session_path.read_text(encoding='utf-8'))
        if not session['attempts']:
            parser.error('No attempt to record an intervention against')
        attempt=session['attempts'][-1]
        interventions=attempt.setdefault('interventions',[])
        if len(interventions)>=2:
            parser.error('Two interventions already recorded; no more allowed')
        interventions.append({'time':time.time(),'returned_to_start_by_operator':True})
        save_session(session_path,session)
        print('Вмешательство записано. Оно не продолжает автономный заезд и не сбрасывает его таймер.')
        return 0
    if not args.start_confirmed:
        parser.error('Physical start confirmation required before motion: --start-confirmed')
    command=[sys.executable,'scripts/mission.py','--drive','--rehearsal','--config',args.config,'--map',args.map]
    if args.operator_cleared_start_gap:
        command.append('--operator-cleared-start-gap')
    if args.operator_accepts_power_faults:
        command.append('--operator-accepts-power-faults')
    if args.action == 'train':
        command += ['--train','--duration','300','--report','data/training_report.json']
        return subprocess.call(command)
    if not args.mission_file:
        parser.error('Supply a separately entered mission each time: --mission-file')
    mission=Path(args.mission_file).read_text(encoding='utf-8').strip()
    if not mission or len(mission)>8000:
        parser.error('Mission text is empty or too long')
    ready=readiness(Path(args.map),Path('data/landmark_templates/catalog.json'))
    if not ready['ready_for_rehearsal']:
        print(json.dumps(ready,ensure_ascii=False,indent=2));return 2
    if not session_path.exists():
        parser.error('Create a field session first: new-session')
    session=json.loads(session_path.read_text(encoding='utf-8'))
    if len(session['attempts'])>=2:
        parser.error('The two attempts for this session have been used')
    number=len(session['attempts'])+1
    folder=Path('data/attempts')/f'{int(session["created_at"])}-{number}'
    folder.mkdir(parents=True,exist_ok=False)
    prompt=folder/'mission.txt';prompt.write_text(mission,encoding='utf-8')
    report=folder/'report.json'
    attempt={'number':number,'started_at':time.time(),'mission_file':str(prompt),'report':str(report),'interventions':[], 'status':'running'}
    session['attempts'].append(attempt);save_session(session_path,session)
    command += ['--mission-file',str(prompt),'--report',str(report)]
    print(f'Попытка {number}/2. Лимит миссии — 300 секунд. Исходный текст:\n{mission}',flush=True)
    try:
        attempt['exit_code']=subprocess.call(command)
        attempt['status']='finished' if attempt['exit_code']==0 else 'failed'
    except KeyboardInterrupt:
        attempt['status']='operator_interrupted'
        attempt['exit_code']=130
    finally:
        attempt['ended_at']=time.time();save_session(session_path,session)
    return attempt['exit_code']


if __name__=='__main__':
    raise SystemExit(main())
