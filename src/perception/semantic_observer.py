"""Local visual descriptions associated with measured lidar ranges."""
import base64
import json
import math
import cv2
import requests
from pathlib import Path
import time
from src.perception.landmark_matcher import LandmarkMatcher


class SemanticObserver:
    def __init__(self, camera_yaws, model='moondream:v2', url='http://127.0.0.1:11434/api/generate', evidence_dir='data/evidence'):
        self.camera_yaws = camera_yaws
        self.model, self.url = model, url
        self.evidence_dir = Path(evidence_dir)
        self.evidence_dir.mkdir(parents=True, exist_ok=True)
        self.templates=[]
        catalog=Path('data/landmark_templates/catalog.json')
        if catalog.exists():
            for item in json.loads(catalog.read_text(encoding='utf-8')):
                reference=cv2.imread(item['image'])
                if reference is None:raise RuntimeError('Missing landmark template')
                if item.get('reference_region'):
                    x0,y0,x1,y1=item['reference_region'];h,w=reference.shape[:2]
                    if not 0<=x0<x1<=1 or not 0<=y0<y1<=1:raise ValueError('Invalid reference region')
                    reference=reference[int(y0*h):int(y1*h),int(x0*w):int(x1*w)]
                self.templates.append((item,LandmarkMatcher(reference)))

    def observe(self, frames, scan, pose, semantic_map, lidar_yaw=180, angle_sign=-1):
        observations = []
        for index, frame in frames.items():
            template_match=None
            if self.templates:
                candidates=[(item,matcher.detect(frame)) for item,matcher in self.templates]
                candidates=[(item,match) for item,match in candidates if match is not None]
                if not candidates:continue
                candidates.sort(key=lambda pair:pair[1]['inliers'],reverse=True)
                if len(candidates)>1 and candidates[0][0].get('identity',candidates[0][0]['image']) != candidates[1][0].get('identity',candidates[1][0]['image']) and candidates[0][1]['inliers']<1.5*candidates[1][1]['inliers']:continue
                template_item,template_match=candidates[0]
            if template_match is not None:
                xs=[point[0] for point in template_match['polygon']]
                if abs(sum(xs)/len(xs)-frame.shape[1]/2)>frame.shape[1]*.1:
                    continue  # No pixel-to-bearing calibration for off-axis objects.
            yaw = self.camera_yaws[index]
            if yaw is None:
                raise ValueError('Camera extrinsics have not been identified')
            # A narrow central crop associates the description with a lidar sector,
            # rather than assigning one range to every object in a wide image.
            h, w = frame.shape[:2]
            crop = frame[:, int(w*0.35):int(w*0.65)]
            ok, encoded = cv2.imencode('.jpg', crop)
            if not ok:
                continue
            distances = [d/1000 for q, a, d in scan if q > 0 and 150 < d < 3500
                         and abs((angle_sign*(a-lidar_yaw)-yaw+180) % 360-180) <= 5]
            if len(distances) < 3:
                continue
            distances.sort()
            distance = distances[len(distances)//2]
            evidence_path = self.evidence_dir / f'camera-{index}-{time.time_ns()}.jpg'
            if not cv2.imwrite(str(evidence_path), frame):
                raise RuntimeError('Cannot save visual evidence')
            started = time.monotonic()
            if template_match is not None:
                description=template_item['description']
            else:
                response = requests.post(self.url, json={'model': self.model,
                    'prompt': 'Describe the main visible physical object in this image in one short sentence. '
                              'Mention its color and whether it is a damaged building, bridge, tree, '
                              'vehicle, debris, person, or wall. Do not invent objects outside the image.',
                    'images': [base64.b64encode(encoded).decode()], 'stream': False,
                    'keep_alive': 0, 'options': {'temperature': 0, 'num_predict': 100,
                                               'num_ctx': 2048, 'num_thread': 2, 'num_batch': 32}}, timeout=(5, 90))
                response.raise_for_status()
                description = response.json().get('response', '').strip()
            with (self.evidence_dir/'vision_log.jsonl').open('a', encoding='utf-8') as log:
                log.write(json.dumps({'time':time.time(),'model':('visual_templates' if template_match is not None else self.model),'camera':index,
                    'image':str(evidence_path),'crop_fraction':[0.35,0.65],
                    'pose':list(pose),'lidar_range_m':distance,'raw_response':description,
                    'duration_s':time.monotonic()-started,'recognition_verified':False,
                    'template_match':template_match},
                    ensure_ascii=False)+'\n')
            if not description or len(description) > 1500:
                continue
            lower = description.lower()
            kinds = [('building', ('building', 'house')), ('bridge', ('bridge',)),
                     ('tree', ('tree',)), ('vehicle', ('car', 'truck', 'vehicle')),
                     ('debris', ('debris', 'rubble')), ('victim', ('person', 'mannequin'))]
            kind = next((k for k, words in kinds if any(word in lower for word in words)), None)
            if kind is None:
                continue
            color = next((c for c in ('yellow', 'red', 'green', 'blue', 'white', 'gray', 'black') if c in lower), 'unknown')
            bearing = pose[2]+math.radians(yaw)
            x, y = pose[0]+distance*math.cos(bearing), pose[1]+distance*math.sin(bearing)
            if not 0 <= x < 4 or not 0 <= y < 4:
                continue
            name = semantic_map.observe(f'{color} {kind}', kind, x, y, 0.6,
                                        description, f'camera_{index}+lidar')
            semantic_map.objects[name].update(category=template_item.get('category') if template_match is not None else None,position_current=True,description=description, evidence_image=str(evidence_path),
                evidence_pose=list(pose), camera=index, lidar_range_m=distance,
                crop_fraction=[0.35, 0.65], recognition_verified=bool(template_match is not None and template_item.get('training_identity_verified')), range_association_verified=False, template_match=template_match, template_identity=(template_item.get('identity') if template_match is not None else None))
            observations.append({'name': name, 'description': description, 'camera': index})
        return observations
