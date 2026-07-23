import cv2
import numpy as np
from enum import Enum

class LandmarkType(Enum):
    FIRE = "fire"              # Красный/оранжевый
    GAS_SPILL = "gas_spill"    # Желтый/коричневый
    FLOOD = "flood"            # Синий/голубой
    COLLAPSE = "collapse"      # Серый/черный
    ASSEMBLY_POINT = "assembly_point"  # Зеленый знак
    GAS_STATION = "gas_station"  # Специфический знак
    BRIDGE = "bridge"          # Горизонтальная структура
    TREE = "tree"              # Вертикальная структура
    VICTIM = "victim"          # Манекен (определенный цвет/форма)

class LandmarkDetector:
    def __init__(self):
        # Цветовые диапазоны в HSV для каждого типа ориентира
        self.color_ranges = {
            LandmarkType.FIRE: ([0, 100, 100], [15, 255, 255]),      # Красный-оранжевый
            LandmarkType.GAS_SPILL: ([20, 100, 100], [35, 255, 255]), # Желтый
            LandmarkType.FLOOD: ([100, 50, 50], [130, 255, 255]),     # Синий
            LandmarkType.ASSEMBLY_POINT: ([40, 50, 50], [70, 255, 255]), # Зеленый
        }
        self.sign_templates = self._load_sign_templates()
    
    def detect_landmarks(self, frame):
        """
        Основной метод детектирования всех ориентиров на кадре
        
        Args:
            frame: кадр с камеры (numpy array)
        
        Returns:
            list of dict: [{'type': LandmarkType, 'bbox': (x,y,w,h), 
                           'confidence': 0.0-1.0, 'distance': meters}]
        """
        landmarks = []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 1. Детектирование по цвету
        for landmark_type, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours = self._find_contours(mask)
            
            for contour in contours:
                bbox = cv2.boundingRect(contour)
                confidence = self._calculate_confidence(contour, bbox)
                
                if confidence > 0.6:  # Порог уверенности
                    distance = self._estimate_distance(bbox)
                    landmarks.append({
                        'type': landmark_type,
                        'bbox': bbox,
                        'confidence': confidence,
                        'distance': distance
                    })
        
        # 2. Детектирование знаков по шаблонам
        signs = self._detect_signs(frame)
        landmarks.extend(signs)
        
        # 3. Детектирование структур (мосты, деревья)
        structures = self._detect_structures(frame)
        landmarks.extend(structures)
        
        return landmarks
    
    def _find_contours(self, mask):
        """Нахождение контуров с фильтрацией по размеру"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        # Фильтрация маленьких контуров
        min_area = 500  # пикселей
        return [c for c in contours if cv2.contourArea(c) > min_area]
    
    def _calculate_confidence(self, contour, bbox):
        """Расчет уверенности детектирования"""
        area = cv2.contourArea(contour)
        bbox_area = bbox[2] * bbox[3]
        
        # Compactness (насколько объект компактен)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return 0
        compactness = 4 * np.pi * area / (perimeter * perimeter)
        
        # Заполненность bbox
        fill_ratio = area / bbox_area if bbox_area > 0 else 0
        
        # Комбинированная уверенность
        confidence = 0.6 * compactness + 0.4 * fill_ratio
        return min(confidence, 1.0)
    
    def _estimate_distance(self, bbox):
        """
        Грубая оценка расстояния до объекта
        (требует калибровки камеры)
        """
        # Чем больше объект в кадре, тем он ближе
        object_width_pixels = bbox[2]
        known_object_width_meters = 0.5  # Примерная ширина объекта
        focal_length = 640  # Для камеры 640x480
        
        distance = (known_object_width_meters * focal_length) / object_width_pixels
        return distance
    
    def _detect_signs(self, frame):
        """Детектирование знаков (Assembly point, Gas station)"""
        signs = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Template matching для известных знаков
        for sign_type, template in self.sign_templates.items():
            res = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
            threshold = 0.8
            loc = np.where(res >= threshold)
            
            for pt in zip(*loc[::-1]):
                bbox = (pt[0], pt[1], template.shape[1], template.shape[0])
                signs.append({
                    'type': sign_type,
                    'bbox': bbox,
                    'confidence': res[pt[1], pt[0]],
                    'distance': self._estimate_distance(bbox)
                })
        
        return signs
    
    def _detect_structures(self, frame):
        """Детектирование структур (мосты, деревья, завалы)"""
        structures = []
        edges = cv2.Canny(frame, 50, 150)
        
        # Детектирование горизонтальных линий (мосты)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, 
                                minLineLength=100, maxLineGap=10)
        
        if lines is not None:
            horizontal_lines = [l for l in lines if abs(l[0][1] - l[0][3]) < 10]
            if len(horizontal_lines) > 5:  # Много горизонтальных линий = мост
                structures.append({
                    'type': LandmarkType.BRIDGE,
                    'bbox': (0, 0, frame.shape[1], frame.shape[0]),
                    'confidence': 0.7,
                    'distance': 5.0  # Примерно
                })
        
        return structures
    
    def _load_sign_templates(self):
        """Загрузка шаблонов знаков (черно-белые изображения)"""
        templates = {}
        templates[LandmarkType.ASSEMBLY_POINT] = cv2.imread('/home/robot/code/input/assembly_sign.png', 0)
        templates[LandmarkType.GAS_STATION] = cv2.imread('/home/robot/code/input/gas_station_sign.png', 0)
        return templates