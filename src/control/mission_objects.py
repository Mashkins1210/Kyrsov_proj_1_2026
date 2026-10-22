"""Regulation object classes; dynamic means movable between attempts."""
import re
import json
from pathlib import Path

STATIC = {"yellow_building", "blue_building", "bridge", "river", "parking"}
DYNAMIC = {"victim", "qr_stand", "traffic_jam", "building_debris", "bus_stop", "tanker", "fallen_tree", "trash_bin"}
LABELS = {
 "yellow_building":"жёлтое здание", "blue_building":"голубое здание (макет-стакан)",
 "bridge":"мост", "river":"река", "parking":"парковка", "victim":"пострадавший",
 "qr_stand":"стойка с QR", "traffic_jam":"автомобильный затор", "building_debris":"обломки здания",
 "bus_stop":"автобусная остановка", "tanker":"бензовоз", "fallen_tree":"упавшее дерево", "trash_bin":"мусорный бак"}


PATTERNS = {
 'yellow_building':r'ж[её]лт\w*\s+(?:разрушенн\w*\s+|обрушивш\w*\s+)?(?:здани\w*|дом\w*)',
 'blue_building':r'(?:голуб\w*|син\w*)\s+(?:здани\w*|дом\w*)',
 'bridge':r'мост\w*','river':r'рек(?:а|и|е|у|ой|ою)', 'parking':r'парков\w*',
 'victim':r'(?:пострадавш|постардавш)\w*','qr_stand':r'стойк\w*\s+с\s+QR(?:-код\w*)?',
 'traffic_jam':r'(?:автомобильн\w*\s+затор\w*|затор\w*|пробк\w*)',
 'building_debris':r'обломк\w*','bus_stop':r'(?:автобусн\w*\s+)?остановк\w*',
 'tanker':r'бензов[оа]з\w*','fallen_tree':r'дерев(?:о|а|ом|у|е|ья|ьев)',
 'trash_bin':r'(?:мусорн\w*\s+)?бак(?:а|ом|у|е|и|ов)?'}


def mentioned_objects(prompt):
    return [{'id':key,'quote':match.group(0)} for key,pattern in PATTERNS.items()
            if (match:=re.search(r'\b'+pattern+r'\b',prompt,re.IGNORECASE))]


def explicit_search_focus(prompt, candidates):
    commands=list(re.finditer(r'(?:найди|найти|ищи|ищите|поиск)\b',prompt,re.IGNORECASE))
    tail=prompt[commands[-1].end():] if commands else ''
    matches=mentioned_objects(tail)
    matches=[item for item in matches if item['id']!='victim']
    if not matches:
        clauses=[part for part in re.split(r'[.!?]',prompt) if re.search(PATTERNS['victim'],part,re.IGNORECASE) and not re.search(r'найди|найти|ищи',part,re.IGNORECASE)]
        if clauses:
            context=clauses[-1]
            matches=[item for item in mentioned_objects(context) if item['id']!='victim']
            matches.sort(key=lambda item:context.find(item['quote']))
    if not matches:
        matches=[item for item in candidates if item['id']!='victim']
        matches.sort(key=lambda item:prompt.find(item['quote']))
    return [matches[0]['id']] if matches else []


def relevant_examples(candidates):
    path=Path(__file__).resolve().parents[2]/'data/mission-training-examples.json'
    if not path.exists():return []
    examples=json.loads(path.read_text(encoding='utf-8'))
    ids={i['id'] for i in candidates}
    examples.sort(key=lambda e:len(ids & {i['id'] for i in e['answer']['objects']}),reverse=True)
    return [{'mission':e['text'],'search_near':e['answer']['search_near'],
             'relations':[[r['subject'],r['relation'],r['object']] for r in e['answer']['relations']]}
            for e in examples[:2]]


def extract_mission(planner, prompt):
    candidates=mentioned_objects(prompt)
    focus=explicit_search_focus(prompt,candidates)
    ids=[item['id'] for item in candidates] or list(LABELS)
    landmarks=[i for i in ids if i!='victim']
    schema={'type':'object','properties':{
        'objects':{'type':'array','items':{'type':'string','enum':ids},'minItems':len(candidates),'maxItems':len(ids),'uniqueItems':True},
        'relations':{'type':'array','maxItems':4,'items':{'type':'array','minItems':3,'maxItems':3,'prefixItems':[
            {'type':'string','enum':ids},{'type':'string','enum':['near','fallen_on','under','among','on','behind','in']},{'type':'string','enum':ids}]}},
        'key_landmarks':{'type':'array','minItems':1,'items':{'type':'string','enum':landmarks}},
        'search_near':{'type':'array','enum':[focus]} if focus else {'type':'array','minItems':1,'items':{'type':'string','enum':landmarks}}},
        'required':['objects','relations','key_landmarks','search_near'],'additionalProperties':False}
    result=planner._ask(
        "Ответ компактный JSON: objects — массив ID; relations — тройки [subject,relation,object]. "
        "Извлеки ВСЕ явно названные объекты из текста миссии, независимо от наличия на карте. "
        "Верни объекты, связи, ключевые ориентиры и объекты search_near, около которых искать пострадавшего. "
        "quote всегда точная непустая цитата исходного текста. Не выдумывай объекты или координаты. "
        "Различай связи: on на, behind за, in в, near около, under под, among среди. "
        "Главное место поиска определяется последней явной командой: возле бензовоза => tanker; "
        "в заторе => traffic_jam; за обломками => building_debris. "
        "Если сказано там, разреши ссылку по описанию положения пострадавшего, а не по последнему слову. "
        "Остановка возле реки: искать у остановки; затор рядом с рекой: искать в заторе. "
        "Постардавший — опечатка слова пострадавший. "
        "Голубой дом представлен стаканом. fallen_tree означает упавшее дерево. "
        "Связь victim near fallen_tree означает пострадавший около дерева. "
        "Связь fallen_tree fallen_on blue_building означает дерево свалилось на голубой дом. "
        "Слова свалившимся на или упавшим на требуют fallen_on, а не near. "
        "У дерева, упавшего на голубой дом: оба являются ориентирами; искать около дерева, "
        "цвет голубой относится к дому, не к дереву. Статичность задаётся регламентом, "
        "динамические объекты могут менять место МЕЖДУ попытками, это не движение во время заезда.",
        {"mission":prompt,"mentioned_objects":candidates,"explicit_search_focus":focus,"examples":relevant_examples(candidates),"instruction":"Examples illustrate relationships only. Never copy objects or quotes from examples. Copy ALL mentioned_objects exactly. No self relationships. Determine relations and search_near from mission. Relations quote must be the original mission verbatim."},schema=schema)
    objects=result.get("objects")
    if isinstance(objects,list) and objects and all(isinstance(i,str) for i in objects):
        quotes={item['id']:item['quote'] for item in candidates}
        result['objects']=[{'id':i,'quote':quotes.get(i,'')} for i in objects]
        relations=result.get('relations',[])
        if not isinstance(relations,list) or any(not isinstance(r,list) or len(r)!=3 for r in relations):
            raise ValueError('Invalid compact relations')
        result['relations']=[{'subject':a,'relation':r,'object':b,'quote':prompt} for a,r,b in relations]
        result['evidence_note']='Object quotes extracted from original text; relation quote references the full original mission'
        objects=result['objects']
    if not isinstance(objects,list) or not objects:raise ValueError("No mission objects extracted")
    ids=set()
    for item in objects:
        if item.get("id") not in LABELS or not item.get("quote") or item["quote"] not in prompt:
            raise ValueError("Mission object lacks source evidence")
        ids.add(item["id"])
        item.update(label=LABELS[item["id"]],mobility="static" if item["id"] in STATIC else "dynamic",
                    requires_fresh_detection=item["id"] in DYNAMIC)
    if candidates and ids != {item["id"] for item in candidates}:
        raise ValueError("LLM omitted or invented an explicitly named object")
    for key in ("key_landmarks","search_near"):
        if not isinstance(result.get(key),list) or any(i not in ids or i=="victim" for i in result[key]):
            raise ValueError("Unsupported mission landmark")
    if not result["key_landmarks"] or not result["search_near"]:raise ValueError("Missing search landmarks")
    if not isinstance(result.get("relations"),list):raise ValueError("Missing relationships")
    if focus and result['search_near']!=focus:raise ValueError('LLM changed the explicit search focus')
    for rel in result["relations"]:
        if rel.get('subject')==rel.get('object'):raise ValueError('Self relationship is not a valid mission relation')
        if rel.get("subject") not in ids or rel.get("object") not in ids or rel.get("relation") not in {"near","fallen_on","under","among","on","behind","in"} or not rel.get("quote") or rel["quote"] not in prompt:
            raise ValueError("Unsupported mission relationship")
    if {'fallen_tree','blue_building'} <= ids and re.search(r'(?:сваливш|упавш)\w*\s+на\s+(?:голуб|син)',prompt,re.IGNORECASE):
        if not any(r['subject']=='fallen_tree' and r['object']=='blue_building' and r['relation']=='fallen_on' for r in result['relations']):
            raise ValueError("LLM failed to preserve fallen-tree-on-building relationship")
    for item in objects:
        item['role']='victim' if item['id']=='victim' else 'primary_landmark' if item['id'] in result['search_near'] else 'supporting_landmark'
        item['is_landmark']=item['id']!='victim'
        item['role_label']={'victim':'ПОСТРАДАВШИЙ','primary_landmark':'ОСНОВНОЙ ОРИЕНТИР ПОИСКА','supporting_landmark':'ДОПОЛНИТЕЛЬНЫЙ ОРИЕНТИР'}[item['role']]
    result['search_constraints']=[{'object_id':i['id'],'mobility':i['mobility'],
        'requires_current_attempt_observation':True} for i in objects if i['id']!='victim']
    result['search_zone_policy']='All stated spatial relationships must be verified; search_near does not discard other landmarks'
    result["dynamic_position_policy"]="Detect again during this attempt; previous coordinates are not confirmation"
    return result


def bind_interpretation(extracted, objects):
    """Bind already validated LLM categories to map entries, without a second inference."""
    focus=extracted['search_near']
    aliases={'yellow_damaged_building':'yellow_building'}
    candidates=[]
    for name,obj in objects.items():
        category=obj.get('category') or aliases.get(obj.get('template_identity'))
        if category not in focus:continue
        if category in DYNAMIC and obj.get('position_current') is not True:continue
        candidates.append((name,obj,category))
    if len(candidates)>1:
        raise ValueError('Multiple matching objects: spatial relationships require current visual confirmation')
    selected=candidates[0] if candidates else None
    category=selected[2] if selected else focus[0]
    kind={'yellow_building':'building','blue_building':'building','fallen_tree':'tree','tanker':'vehicle','traffic_jam':'vehicle','building_debris':'debris','qr_stand':'unknown'}.get(category,category)
    color='yellow' if category=='yellow_building' else 'blue' if category=='blue_building' else 'unknown'
    return {'target':selected[0] if selected else None,'destination':'start',
            'reason':'Category binding from validated local LLM extraction; no repeated inference',
            'search_description':LABELS[category],
            'target_evidence':selected[1].get('description','') if selected else '',
            'requirements':{'color':color,'kind':kind,'condition':'unknown'},
            'search_constraints':extracted.get('search_constraints',[]),
            'relations':extracted['relations'],'spatial_relationships_verified':False}
