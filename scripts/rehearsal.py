"""One five-minute rehearsal: operator layout, real visual search, QR, return."""
import argparse
import json
import math
import os
import tempfile
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.mission import MissionApp, load_settings
from src.control.semantic_map import SemanticMap


def save_session(path, session):
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(prefix=path.name+'.',suffix='.tmp',dir=path.parent)
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as output:
            json.dump(session,output,ensure_ascii=False,indent=2)
            output.flush()
            os.fsync(output.fileno())
        os.replace(name,path)
    finally:
        if os.path.exists(name):os.unlink(name)


def load_session(path):
    path=Path(path)
    if not path.exists():return {'created_at':time.time(),'attempts':[]}
    try:
        session=json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(session,dict) or not isinstance(session.get('attempts'),list):
            raise ValueError('Invalid session structure')
        if any(not isinstance(attempt,dict) for attempt in session['attempts']):
            raise ValueError('Invalid attempt records')
        return session
    except (ValueError,UnicodeError) as exc:
        raise ValueError(f'Session file {path} is empty or damaged; restore its records or select a separate rehearsal session with --session. Attempt counters were not reset.') from exc


def seed_map(layout, catalog):
    m=SemanticMap()
    for number,item in enumerate(layout['landmarks']):
        cells=item['cells']
        if not cells or any(len(cell)!=2 or any(type(v) is not int or not 0<=v<5 for v in cell) for cell in cells):
            raise ValueError('Invalid operator landmark cells [x,y]')
        label=item['label']
        if label=='cup':label='blue building'
        category=item.get('category') or ('blue_building' if 'blue' in label and 'building' in label else 'yellow_building' if 'yellow' in label and 'building' in label else label if label in ('bridge','river','parking') else None)
        kind='building' if 'building' in label else label if label in ('bridge','river') else 'unknown'
        template=next((t for t in catalog if t.get('category')==category and category is not None or category=='yellow_building' and t.get('identity')=='yellow_damaged_building'),None)
        name=template['identity'] if template else f'{kind}_{number}'
        x=sum((c[0]+.5)*.8 for c in cells)/len(cells)
        y=sum((c[1]+.5)*.8 for c in cells)/len(cells)
        m.add_object(name,x,y,kind,category=category,label=label,description=template['description'] if template else label,
                     cells_xy=cells,coordinate_source='operator layout',recognition_verified=False,
                     template_identity=template['identity'] if template else None)
    return m


def return_budget(app):
    _,pose=app.nav.sense()
    route=app.map.grid.route(pose[:2],tuple(app.map.start_pose[:2]),app.nav.radius)
    if route is None:
        raise RuntimeError('No measured safe return route; outward travel stopped')
    points=[tuple(pose[:2])]+app.nav.shortcut_route(route,pose[:2],tuple(app.map.start_pose[:2]))
    length=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
    turns=max(0,len(points)-2)
    return max(60,length/.08+turns*15+55)


def detect_target(app, object_id):
    import cv2
    target=app.map.objects[object_id]
    for item,matcher in app.observer.templates:
        if item.get('identity')!=target.get('template_identity') or not item.get('training_identity_verified'):
            continue
        for camera,frame in app.camera.get_frames().items():
            match=matcher.detect(frame)
            if match is None:
                continue
            import numpy as np
            annotated=frame.copy();polygon=np.asarray(match['polygon'],dtype=np.int32)
            cv2.polylines(annotated,[polygon],True,(0,255,0),3)
            cv2.putText(annotated,object_id,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,0),2)
            path=Path('data/evidence')/f'detected-{time.time_ns()}.jpg'
            if not cv2.imwrite(str(path),annotated):raise RuntimeError('Cannot save detection evidence')
            target.update(recognition_verified=True,evidence_image=str(path),camera=camera)
            app.event('object_detected',object_id=object_id,camera=camera,evidence_image=str(path),
                      match=match,coordinate_source='operator layout, not image-derived range')
            return True
    return False


def next_station(app,target,visited):
    _,pose=app.nav.sense();grid=app.map.grid
    candidates=[]
    for row in range(8,grid.n-7,8):
        for col in range(8,grid.n-7,8):
            point=grid.point((row,col));distance=math.dist(point,pose[:2])
            if point in visited or not .2<=distance<=.85 or not grid.is_position_clear(point,app.nav.radius):continue
            turn=abs(math.atan2(math.sin(math.atan2(point[1]-pose[1],point[0]-pose[0])-pose[2]),
                               math.cos(math.atan2(point[1]-pose[1],point[0]-pose[0])-pose[2])))
            unknown=~grid.observed[max(0,row-8):row+9,max(0,col-8):col+9]
            score=(math.dist(point,(target['x'],target['y'])) if target else -float(unknown.mean()))+.3*distance+.05*turn
            candidates.append((score,point))
    for _,point in sorted(candidates):
        if grid.route(pose[:2],point,app.nav.radius):return point
    return None


def execute(app,text,result):
    app.event('mission_received',mission=text,llm='LOCAL qwen2.5:1.5b',deadline_seconds=300)
    app.planner.request_timeout=20
    app.event('mission_interpreted',mission=text,interpretation=result)
    name=result.get('target');target=app.map.objects.get(name)
    if target:
        app.event('landmark_selected',object_id=name,coordinates=[target['x'],target['y']],description=target['description'],coordinate_source='prior; fresh visual confirmation pending')
    else:
        app.event('landmark_search_started',requirements=result['requirements'],coordinates_known=False,relations=result['relations'])
    visited=set();found=False
    while time.monotonic()<app.mission_deadline-return_budget(app)-75:
        app.nav.phase_deadline=app.mission_deadline-return_budget(app)-30
        app.motor.stop_all()
        app.observe_station()
        if target is None:
            from src.control.mission_objects import bind_interpretation
            resolved=bind_interpretation(result,app.map.objects)
            name=resolved.get('target');target=app.map.objects.get(name)
        if target is not None and detect_target(app,name):found=True;break
        point=next_station(app,target,visited)
        if point is None:
            app.event('search_waiting_for_safe_route');app.nav.sense();time.sleep(.3);continue
        visited.add(point)
        app.event('search_waypoint',goal=list(point),remaining_s=app.mission_deadline-time.monotonic())
        app.nav.navigate(point,timeout=min(35,app.mission_deadline-time.monotonic()-return_budget(app)-75))
        app.map.save('data/live/measured-map.json')
    if not found:raise RuntimeError('Landmark not visually detected within the search budget')
    reserve=return_budget(app)
    if app.mission_deadline-time.monotonic()<reserve+60:raise RuntimeError('Not enough time for victim QR and return')
    app.nav.phase_deadline=app.mission_deadline-reserve-30
    app.confirm_landmark_arrival(name,target)
    app.find_victim_qr(target,timeout=max(1,min(60,app.mission_deadline-time.monotonic()-return_budget(app)-15)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission-file')
    parser.add_argument('--check-only',action='store_true')
    parser.add_argument('--session',default='data/final-session.json')
    parser.add_argument('--drive',action='store_true')
    parser.add_argument('--unlimited-rehearsals',action='store_true',help='Practice only: keep history without the official two-attempt limit')
    parser.add_argument('--start-confirmed',action='store_true')
    parser.add_argument('--operator-accepts-power-faults',action='store_true')
    parser.add_argument('--operator-cleared-start-gap',action='store_true')
    args=parser.parse_args()
    if not args.check_only and (not args.drive or not args.start_confirmed):parser.error('Explicit --drive and --start-confirmed required')
    if not args.check_only and not args.mission_file:parser.error('A separately entered mission file is required for each run')
    from scripts.validate_mission_text import read_mission_text
    try:text=read_mission_text(args.mission_file) if args.mission_file else None
    except ValueError as exc:parser.error(str(exc))
    settings=load_settings('config/robot.json')
    settings.update(rehearsal=True,scan_localization=True,drive_speed=180,turn_speed=100,
                    operator_accepts_power_faults=args.operator_accepts_power_faults,
                    operator_cleared_start_gap=args.operator_cleared_start_gap,
                    absolute_deadline=time.monotonic()+300)
    m=seed_map(json.loads(Path('data/operator-layout.json').read_text()),
               json.loads(Path('data/landmark_templates/catalog.json').read_text()))
    from scripts.mission import validate_drive
    validate_drive(settings)
    if args.check_only:
        print(json.dumps({'motion_started':False,'software_configuration_valid':True,'start_cell_xy':settings['start_cell'],'heading_deg':settings['start_heading_deg'],'drive_speed':settings['drive_speed'],'global_localization_verified':settings.get('localization_verified'),'coordinate_source':'operator layout','prompt_required_for_run':True,'landmarks':m.objects},ensure_ascii=False,indent=2))
        return 0
    session_path=Path(args.session)
    try:session=load_session(session_path)
    except ValueError as exc:parser.error(str(exc))
    if not args.unlimited_rehearsals and len(session['attempts'])>=2:parser.error('Two attempts already used in this session; do not reset during an official trial')
    from src.control.llm_planner import LLMPlanner
    from src.control.mission_objects import extract_mission, bind_interpretation
    planner=LLMPlanner();planner.request_timeout=min(60,max(1,settings['absolute_deadline']-time.monotonic()-120))
    try:
        extracted=extract_mission(planner,text)
    except Exception as exc:
        parser.error(f'Mission extraction failed before motor initialization: {exc}')
    processed_path=Path(args.mission_file).with_suffix('.processed.json')
    save_session(processed_path,{'mission_text':text,'interpretation':extracted,'source':'local LLM','detection_confirmed':False})
    save_session(Path('data/live/mission-objects.json'),extracted)
    print('ИСХОДНЫЙ ПРОМПТ:\n'+text,flush=True)
    print('ТРАКТОВКА ЛОКАЛЬНОЙ LLM:\n'+json.dumps(extracted,ensure_ascii=False,indent=2),flush=True)
    print('ФАЙЛ ОБРАБОТАННОЙ ИНФОРМАЦИИ: '+str(processed_path.resolve()),flush=True)
    print('КОПИЯ ДЛЯ ЭКРАНА: '+str(Path('data/live/mission-objects.json').resolve()),flush=True)
    # Resolve the requested target before constructing/arming any hardware.
    try:
        prepared=bind_interpretation(extracted,m.objects)
        target=m.objects.get(prepared.get('target'))
        prepared.update(search_near=extracted['search_near'])
        if target and target.get('category') not in extracted['search_near']:
            raise ValueError('Selected landmark disagrees with the extracted victim search location')
        if time.monotonic()>settings['absolute_deadline']-120:
            raise ValueError('Less than 120 seconds remain for motion and return; start cancelled')
    except Exception as exc:
        save_session(Path(args.mission_file).with_suffix('.validation.json'),{'mission':text,'extraction':extracted,'motion_started':False,'error':str(exc)})
        parser.error(f'Mission validation failed before motor initialization: {exc}')
    print(json.dumps({'selected_landmark':prepared},ensure_ascii=False,indent=2),flush=True)
    attempt={'number':len(session['attempts'])+1,'mission_text':text,'started_at':time.time(),'interventions':0,'status':'running','unlimited_rehearsal':args.unlimited_rehearsals}
    session['attempts'].append(attempt);session_path.parent.mkdir(parents=True,exist_ok=True);save_session(session_path,session)
    live=Path('data/live');live.mkdir(parents=True,exist_ok=True)
    for old in ('report.json','report.html'):
        old_path=live/old
        if old_path.exists():old_path.rename(live/f'{int(time.time_ns())}-{old}')
    app=None;failure=None;returned=False;cancelled=False
    try:
        app=MissionApp(settings,m,'data/live/report.json')
        app.event('mission_objects_extracted',mission=text,extraction=extracted)
        app.event('test_notice',message='ПРОБНЫЙ ЗАЕЗД. Координаты объектов заданы оператором; обнаружение проверяется свежими кадрами. Глобальная точность локализации ещё не подтверждена.')
        execute(app,text,prepared)
    except BaseException as exc:
        cancelled=isinstance(exc,KeyboardInterrupt)
        failure=str(exc) or type(exc).__name__
        if app:app.motor.stop_all();app.event('mission_error',error=failure)
    finally:
        if app:
            try:
                if cancelled:
                    raise RuntimeError('Operator stop: return motion cancelled')
                app.nav.phase_deadline=None
                app.event('return_started',remaining_s=app.mission_deadline-time.monotonic())
                app.nav.navigate(tuple(app.map.start_pose[:2]),timeout=max(0,app.mission_deadline-time.monotonic()-15))
                app.nav.rotate_to(app.map.start_pose[2],timeout=max(0,app.mission_deadline-time.monotonic()))
                _,pose=app.nav.sense()
                returned=math.dist(pose[:2],app.map.start_pose[:2])<=.1 and abs(math.atan2(math.sin(pose[2]-app.map.start_pose[2]),math.cos(pose[2]-app.map.start_pose[2])))<=.1
                if not returned:raise RuntimeError('Starting position or heading not restored')
                app.event('return_completed',pose=pose,cell_xy=settings['start_cell'])
            except BaseException as exc:
                failure=f'{failure or ""}; return: {exc}'
            app.report['success']=not failure and returned and app.report.get('victim') is not None
            app.event('mission_completed' if app.report['success'] else 'failed',error=failure,returned_to_start=returned)
            app.shutdown()
            attempt.update(status='completed' if app.report['success'] else 'failed',ended_at=time.time(),success=app.report['success'],returned_to_start=returned,error=failure,shutdown_errors=app.motor.shutdown_errors)
            save_session(session_path,session)
            app.map.save(live/'measured-map.json')
        else:
            attempt.update(ended_at=time.time(),success=False,error=failure,status='initialization_failed')
            save_session(session_path,session)
    return 0 if app and app.report['success'] else 1


if __name__=='__main__':
    import signal
    def operator_stop(signum,frame):
        raise KeyboardInterrupt('Operator termination')
    signal.signal(signal.SIGTERM,operator_stop)
    raise SystemExit(main())
