"""Self-contained readable judging log: no invented scores or coordinates."""
import base64
import html
import json
from pathlib import Path


def write_report(report, semantic_map, filename):
    esc = lambda value: html.escape(str(value))
    events = report.get('events', [])
    interpretation = next((e for e in reversed(events) if e['kind'] == 'mission_interpreted'), {})
    plan = interpretation.get('interpretation', {})
    target = plan.get('target')
    selected = semantic_map.objects.get(target, {})
    received = next((e for e in reversed(events) if e['kind']=='mission_received'), {})
    mission = interpretation.get('mission', received.get('mission','Ожидается новый промпт и подтверждение старта'))
    image = ''
    path = selected.get('evidence_image')
    if path and Path(path).is_file():
        encoded = base64.b64encode(Path(path).read_bytes()).decode()
        image = f'<img alt="Реальное наблюдение выбранного ориентира" src="data:image/jpeg;base64,{encoded}">'
    cells = []
    for row in reversed(range(5)):
        for col in range(5):
            names = [name for name, obj in semantic_map.objects.items() if int(obj['x']/0.8) == col and int(obj['y']/0.8) == row]
            chosen = target in names
            cells.append(f'<div class="cell {"chosen" if chosen else ""}"><small>({col},{row})</small><br>{"<br>".join(esc(n) for n in names)}</div>')
    status = 'Возвращение и QR подтверждены программой; требуется проверка судьями' if report.get('success') else 'Миссия не завершена'
    if report.get('rehearsal'):
        status += ' · ПРОБНЫЙ ЗАЕЗД: глобальная точность локализации не сертифицирована'
    notice = next((e.get('message','') for e in events if e['kind'] == 'test_notice'), '')
    req = plan.get('requirements', {})
    colors = {'yellow':'жёлтый','red':'красный','blue':'голубой/синий','unknown':'не указан'}
    kinds = {'building':'здание','bridge':'мост','tree':'дерево','vehicle':'транспорт','debris':'обломки','victim':'пострадавший','unknown':'не указан'}
    meaning = f'Цвет: {colors.get(req.get("color"), req.get("color","—"))}; объект: {kinds.get(req.get("kind"),req.get("kind","—"))}; разрушение: {"требуется" if req.get("condition") == "damaged" else "не указано"}. После поиска — остановка минимум на 5 секунд, считывание QR и возврат на старт.'
    checks = [('mission_interpreted','LLM интерпретировала исходный текст'),('landmark_found','Ориентир найден: новое изображение при прибытии'),('victim_stop_confirmed','Остановка у пострадавшего не менее 5 секунд'),('qr_confirmed_after_stop','Состояние считано из свежего QR после остановки'),('mission_completed','Возврат в стартовую ячейку и направление')]
    stages = ''.join(f'<li>{esc(label)}: {"подтверждено программой" if any(e["kind"]==kind for e in events) else "не подтверждено"}</li>' for kind,label in checks)
    cards = []
    detection_cards = []
    for event in events:
        if event['kind'] not in ('object_detected','landmark_found'):
            continue
        detection_path=event.get('evidence_image')
        if detection_path and Path(detection_path).is_file():
            encoded=base64.b64encode(Path(detection_path).read_bytes()).decode()
            detection_cards.append(f'<p>Обнаружено: {esc(event.get("object_id"))}; камера {esc(event.get("camera"))}</p><img alt="Реальное обнаружение с рамкой" src="data:image/jpeg;base64,{encoded}">')
    detection_gallery=''.join(detection_cards) or '<p>Ни одного объекта ещё не подтверждено свежим изображением.</p>'
    cards = []
    for event in events:
        data = {key: value for key, value in event.items() if key != 'time'}
        cards.append(f'<details open><summary>{esc(event["kind"])}</summary><pre>{esc(json.dumps(data, ensure_ascii=False, indent=2))}</pre></details>')
    extraction=next((e.get('extraction',{}) for e in reversed(events) if e['kind']=='mission_objects_extracted'),{})
    extracted_rows=''.join(f"<li>{esc(item.get('label'))}: {'статичный' if item.get('mobility')=='static' else 'динамический между попытками'}; цитата: {esc(item.get('quote'))}</li>" for item in extraction.get('objects',[]))
    extraction_html=f"<h3>Объекты исходного промпта</h3><ul>{extracted_rows}</ul><p>Ключевые ориентиры: {esc(', '.join(extraction.get('key_landmarks',[])))}. Искать около: {esc(', '.join(extraction.get('search_near',[])))}.</p><pre>{esc(json.dumps(extraction.get('relations',[]),ensure_ascii=False,indent=2))}</pre><p>Извлечение из текста не подтверждает обнаружение камерой. Динамические объекты требуют нового наблюдения в этой попытке.</p>" if extraction else ''
    victim=report.get('victim') or {}
    latest_qr=next((e for e in reversed(events) if e['kind'] in ('qr_detected','qr_read','qr_confirmed_after_stop')),{})
    qr_text=victim.get('raw_qr',latest_qr.get('raw','QR ещё не считан'))
    qr_status=victim.get('status','Состояние ещё не подтверждено')
    content = f'''<!doctype html><html lang="ru"><meta charset="utf-8"><meta http-equiv="refresh" content="3"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Эвакуация — доказательства выполнения</title><style>
body{{font:16px system-ui;background:#eef2f7;color:#142235;margin:0;padding:24px}}main{{max-width:1100px;margin:auto}}
section,details{{background:white;padding:20px;margin:16px 0;border-radius:12px}}h1{{font-size:28px}}h2{{font-size:20px}}
.grid{{display:grid;grid-template-columns:repeat(5,1fr);gap:4px}}.cell{{min-height:90px;padding:6px;background:#e9edf3;overflow-wrap:anywhere}}
.chosen{{background:#ffdf78;border:2px solid #8d5f00}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}img{{max-width:100%;max-height:480px}}
.status{{font-weight:bold}}small{{color:#526479}}</style><main>
<h1>Хакатон «Эвакуация»</h1><p class="status">{esc((notice+' · ' if notice else '')+status)}</p>
<section><h2>1. Исходный текст и понимание миссии</h2><blockquote>{esc(mission)}</blockquote>
<p>{esc(meaning)}</p>{extraction_html}
<pre>{esc(json.dumps(plan, ensure_ascii=False, indent=2))}</pre></section>
<section><h2>2. Наблюдаемый ориентир</h2>{detection_gallery}<p>Выбран: {esc(target)}. Жёлтая ячейка — выбранный объект. Координаты [x,y]: x вправо, y вверх. Индексы 0..4, размеры ячейки 800×800 мм.</p>
<div class="grid">{''.join(cells)}</div><pre>{esc(json.dumps(selected, ensure_ascii=False, indent=2))}</pre>{image}
<p>Цитата описания подтверждает связь с наблюдением. Правильность распознавания подтверждается сопоставлением снимка и реального объекта, а не уверенностью LLM.</p></section>
<section><h2>3. QR, остановка и возвращение</h2><h3>Считанный текст QR</h3><pre>{esc(qr_text)}</pre><p>Состояние пострадавшего: {esc(qr_status)}</p><ul>{stages}</ul><pre>{esc(json.dumps(report.get('victim'), ensure_ascii=False, indent=2))}</pre>
<p>Успех отмечается после подтверждённого QR, остановки не менее 5 секунд и возврата на старт. Баллы автоматически не присваиваются.</p></section>
<section><h2>4. Журнал решений и измерений</h2>{''.join(cards)}</section></main></html>'''
    output = Path(filename)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(content, encoding='utf-8')
