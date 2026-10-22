import json
import math
import os
from pathlib import Path
from src.control.grid_map import GridMap


class SemanticMap:
    def __init__(self):
        self.objects = {}
        self.robot_pose = (0.0, 0.0, 0.0)
        self.grid = GridMap()
        self.start_pose = None
        self.training_complete = False

    def add_object(self, name, x, y, obj_type='unknown', **metadata):
        if not isinstance(name, str) or not name or not all(math.isfinite(v) for v in (x, y)):
            raise ValueError('Invalid semantic object')
        self.objects[name] = {'x': float(x), 'y': float(y), 'type': obj_type, **metadata}
        self.objects[name]['mobility']=self.mobility(self.objects[name])

    @staticmethod
    def mobility(obj):
        from src.control.mission_objects import STATIC,DYNAMIC
        category=obj.get('category',obj.get('template_identity'))
        if category in STATIC:return 'static'
        if category in DYNAMIC:return 'dynamic'
        if obj.get('type') in ('tree','vehicle','debris','victim','trash_bin','bus_stop','qr_stand','traffic_jam'):
            return 'dynamic'
        if obj.get('type') in ('building','bridge','river','parking'):return 'static'
        return 'unknown'

    def begin_attempt(self):
        for obj in self.objects.values():
            obj['mobility']=self.mobility(obj)
            if obj['mobility'] in ('dynamic','unknown'):
                obj['previous_position']={'x':obj['x'],'y':obj['y']}
                obj['position_current']=False
                obj['recognition_verified']=False
                obj['range_association_verified']=False
                obj['observations']=0

    def observe(self, label, obj_type, x, y, confidence, description, source):
        stale=[(name,obj) for name,obj in self.objects.items()
               if obj.get('label')==label and obj.get('position_current') is False]
        # A single stale identity may move arbitrarily between attempts. Never average
        # its old location into the current measurement; multiple identities stay ambiguous.
        if len(stale)==1:
            name,obj=stale[0]
            obj.update(x=x,y=y,observations=1,confidence=confidence,description=description,
                       source=source,position_current=True,recognition_verified=False)
            return name
        # Merge spatially close sightings rather than inventing object IDs every frame.
        for name, obj in self.objects.items():
            if obj.get('label') == label and math.hypot(obj['x']-x, obj['y']-y) < 0.4:
                count = obj.get('observations', 1)
                obj['x'] = (obj['x']*count+x)/(count+1)
                obj['y'] = (obj['y']*count+y)/(count+1)
                obj['observations'] = count+1
                obj['confidence'] = max(confidence, obj.get('confidence', 0))
                return name
        name = f'{obj_type}_{len(self.objects)}'
        self.add_object(name, x, y, obj_type, label=label, description=description,
                        confidence=confidence, observations=1, source=source,
                        recognition_verified=False,position_current=True)
        return name

    def get_coordinates(self, name):
        if isinstance(name, str) and name.startswith('cell_'):
            parts = name.split('_')
            if len(parts) == 3 and all(len(p) == 1 and p in '01234' for p in parts[1:]):
                row, col = map(int, parts[1:])
                return {'x': (col+0.5)*0.8, 'y': (row+0.5)*0.8, 'type': 'cell'}
        return self.objects.get(name)

    def update_robot_pose(self, x, y, theta):
        self.robot_pose = (x, y, theta)

    def distance_to(self, name):
        obj = self.get_coordinates(name)
        return math.hypot(obj['x']-self.robot_pose[0], obj['y']-self.robot_pose[1]) if obj else float('inf')

    def save(self, filename='semantic_map.json'):
        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix+'.tmp')
        temporary.write_text(json.dumps({'version': 2, 'objects': self.objects,
            'robot_pose': self.robot_pose, 'start_pose': self.start_pose,
            'training_complete': self.training_complete, 'grid': self.grid.save_data()},
            ensure_ascii=False, indent=2), encoding='utf-8')
        os.replace(temporary, path)

    def load(self, filename='semantic_map.json'):
        data = json.loads(Path(filename).read_text(encoding='utf-8'))
        if data.get('version') != 2:
            raise ValueError('Legacy static map is not a trained occupancy/semantic map')
        self.objects = data['objects']
        for obj in self.objects.values():
            if not all(math.isfinite(obj[v]) and 0 <= obj[v] <= 4 for v in ('x', 'y')):
                raise ValueError('Invalid semantic map coordinates')
        self.begin_attempt()
        self.robot_pose = tuple(data['robot_pose'])
        self.start_pose = data['start_pose']
        self.training_complete = bool(data['training_complete'])
        self.grid = GridMap.from_data(data['grid'])
