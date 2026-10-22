import cv2
import numpy as np
from enum import Enum
from typing import List, Dict, Tuple, Optional
import os
import warnings

# Подавляем предупреждения Qt о шрифтах в виртуальном окружении
warnings.filterwarnings("ignore", message=".*QFontDatabase.*")


class LandmarkType(Enum):
    FIRE = "fire"
    GAS_SPILL = "gas_spill"
    FLOOD = "flood"
    COLLAPSE = "collapse"
    ASSEMBLY_POINT = "assembly_point"
    GAS_STATION = "gas_station"
    BRIDGE = "bridge"
    TREE = "tree"
    VICTIM = "victim"


class LandmarkDetector:
    def __init__(self):
        # Цветовые диапазоны HSV
        self.color_ranges = {
            LandmarkType.FIRE: ([0, 100, 100], [15, 255, 255]),
            LandmarkType.GAS_SPILL: ([20, 100, 100], [35, 255, 255]),
            LandmarkType.FLOOD: ([100, 50, 50], [130, 255, 255]),
            LandmarkType.ASSEMBLY_POINT: ([40, 50, 50], [70, 255, 255]),
        }

        # HOG параметры
        try:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.hog_win_stride = (8, 8)
            self.hog_padding = (16, 16)
            self.hog_scale = 1.05
            self.hog_threshold = 0.3
            self.has_hog = True
        except AttributeError:
            print("⚠️ HOGDescriptor недоступен. Детектирование жертв отключено.")
            self.has_hog = False

        # Параметры детектирования завалов (БЕЗ GLCM, только OpenCV)
        # Завалы характеризуются высокой плотностью хаотичных границ
        self.collapse_edge_density_threshold = 0.15  # Порог плотности границ
        self.min_collapse_area = 3000                # Минимальная площадь завала
        self.collapse_window_size = 64               # Размер окна анализа
        
        # Загрузка шаблонов с проверкой существования
        self.sign_templates = self._load_sign_templates()

    def _load_sign_templates(self) -> Dict[LandmarkType, Optional[np.ndarray]]:
        """Безопасная загрузка шаблонов знаков."""
        templates = {}
        sign_files = {
            LandmarkType.ASSEMBLY_POINT: '/home/robot/code/input/assembly_sign.png',
            LandmarkType.GAS_STATION: '/home/robot/code/input/gas_station_sign.jpg',
        }

        for l_type, path in sign_files.items():
            if os.path.exists(path):
                template = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if template is not None:
                    templates[l_type] = template
                else:
                    print(f"❌ Ошибка чтения файла: {path}")
            else:
                print(f"⚠️ Файл шаблона не найден: {path}")

        return templates

    def detect_landmarks(self, frame: np.ndarray, scan=None) -> List[dict]:
        """Основной метод детектирования всех ориентиров на кадре"""
        landmarks = []

        if frame is None or frame.size == 0:
            return landmarks

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 1. Детектирование по цвету
        for landmark_type, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours = self._find_contours(mask)

            for contour in contours:
                bbox = cv2.boundingRect(contour)
                confidence = self._calculate_confidence(contour, bbox)

                if confidence > 0.6:
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

        # 4. Детектирование манекена (HOG)
        if self.has_hog:
            victims = self._detect_victim(frame)
            landmarks.extend(victims)

        # 5. Детектирование завалов (Анализ плотности границ - БЕЗ GLCM)
        # Texture-only rubble classifier is not implemented; do not claim a detection.

        return landmarks

    def _find_contours(self, mask):
        """Нахождение контуров с фильтрацией по размеру"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        min_area = 500
        return [c for c in contours if cv2.contourArea(c) > min_area]

    def _calculate_confidence(self, contour, bbox):
        """Расчет уверенности детектирования"""
        area = cv2.contourArea(contour)
        bbox_area = bbox[2] * bbox[3]

        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return 0
        compactness = 4 * np.pi * area / (perimeter * perimeter)

        fill_ratio = area / bbox_area if bbox_area > 0 else 0

        confidence = 0.6 * compactness + 0.4 * fill_ratio
        return min(confidence, 1.0)

    def _estimate_distance(self, bbox):
        """Грубая оценка расстояния до объекта"""
        object_width_pixels = bbox[2]
        known_object_width_meters = 0.5
        focal_length = 640

        if object_width_pixels <= 0:
            return float('inf')
            
        distance = (known_object_width_meters * focal_length) / object_width_pixels
        return distance

    def _detect_signs(self, frame):
        """Детектирование знаков (Assembly point, Gas station)"""
        signs = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        for sign_type, template in self.sign_templates.items():
            if template.shape[0] > gray.shape[0] or template.shape[1] > gray.shape[1]:
                continue
            res = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
            threshold = 0.8
            loc = np.where(res >= threshold)

            for pt in zip(*loc[::-1]):
                bbox = (pt[0], pt[1], template.shape[1], template.shape[0])
                signs.append({
                    'type': sign_type,
                    'bbox': bbox,
                    'confidence': float(res[pt[1], pt[0]]),
                    'distance': self._estimate_distance(bbox)
                })

        return signs

    def _detect_structures(self, frame):
        """Улучшенное детектирование структур"""
        structures = []
        edges = cv2.Canny(frame, 50, 150)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100,
                                minLineLength=100, maxLineGap=10)
        if lines is not None:
            horizontal_lines = [l for l in lines if abs(l[0][1] - l[0][3]) < 10]
            vertical_lines = [l for l in lines if abs(l[0][0] - l[0][2]) < 10]

            if len(horizontal_lines) > 5 and len(vertical_lines) >= 2:
                all_coords = []
                for line in horizontal_lines + vertical_lines:
                    all_coords.extend([line[0][0], line[0][1], line[0][2], line[0][3]])

                x_min = min(all_coords[::2])
                y_min = min(all_coords[1::2])
                x_max = max(all_coords[::2])
                y_max = max(all_coords[1::2])

                bbox = (x_min, y_min, x_max - x_min, y_max - y_min)

                width_pixels = bbox[2]
                distance = (2.0 * 640) / width_pixels if width_pixels > 0 else 10.0

                structures.append({
                    'type': LandmarkType.BRIDGE,
                    'bbox': bbox,
                    'confidence': min(len(horizontal_lines) / 20, 1.0),
                    'distance': distance
                })
        return structures

    # ==========================================
    # ДЕТЕКТИРОВАНИЕ МАНЕКЕНА (HOG)
    # ==========================================
    def _detect_victim(self, frame: np.ndarray) -> List[dict]:
        """Детектирование манекена с помощью HOG-дескриптора."""
        victims = []

        if frame is None or frame.size == 0:
            return victims

        original_height, original_width = frame.shape[:2]

        scale_factor = 1.0
        if original_width > 640:
            scale_factor = 640.0 / original_width
            frame_resized = cv2.resize(
                frame,
                (int(original_width * scale_factor),
                 int(original_height * scale_factor))
            )
        else:
            frame_resized = frame

        boxes_standing, _ = self.hog.detectMultiScale(
            frame_resized,
            winStride=self.hog_win_stride,
            padding=self.hog_padding,
            scale=self.hog_scale,
            useMeanshiftGrouping=False
        )

        frame_rotated = cv2.rotate(frame_resized, cv2.ROTATE_90_CLOCKWISE)
        boxes_lying, _ = self.hog.detectMultiScale(
            frame_rotated,
            winStride=self.hog_win_stride,
            padding=self.hog_padding,
            scale=self.hog_scale,
            useMeanshiftGrouping=False
        )

        boxes_lying_converted = []
        for (x, y, w, h) in boxes_lying:
            h_rot, w_rot = frame_rotated.shape[:2]
            new_x = y
            new_y = frame_resized.shape[0] - x - w
            new_w = h
            new_h = w
            boxes_lying_converted.append((new_x, new_y, new_w, new_h))

        all_boxes = []
        all_weights = []

        for box in boxes_standing:
            all_boxes.append(tuple(box))
            all_weights.append(1.0)

        for box in boxes_lying_converted:
            all_boxes.append(box)
            all_weights.append(0.9)

        if len(all_boxes) == 0:
            return victims

        boxes_array = np.array(all_boxes)
        indices = cv2.dnn.NMSBoxes(
            boxes_array.tolist(),
            all_weights,
            score_threshold=self.hog_threshold,
            nms_threshold=0.4
        )

        for idx in np.asarray(indices, dtype=int).reshape(-1):
            x, y, w, h = boxes_array[idx]

            x_orig = int(x / scale_factor)
            y_orig = int(y / scale_factor)
            w_orig = int(w / scale_factor)
            h_orig = int(h / scale_factor)

            bbox = (x_orig, y_orig, w_orig, h_orig)

            confidence = self._calculate_victim_confidence(
                bbox, original_width, original_height
            )

            distance = self._estimate_victim_distance(bbox)

            victims.append({
                'type': LandmarkType.VICTIM,
                'bbox': bbox,
                'confidence': confidence,
                'distance': distance,
                'is_lying': idx >= len(boxes_standing),
                'size_pixels': w_orig * h_orig
            })

        return victims

    def _calculate_victim_confidence(
        self,
        bbox: Tuple[int, int, int, int],
        frame_width: int,
        frame_height: int
    ) -> float:
        """Расчет уверенности детектирования манекена."""
        x, y, w, h = bbox

        aspect_ratio = h / w if w > 0 else 0
        if 1.5 <= aspect_ratio <= 4.0:
            ratio_score = 1.0
        elif 1.0 <= aspect_ratio <= 5.0:
            ratio_score = 0.7
        else:
            ratio_score = 0.3

        if 50 <= h <= 400:
            size_score = 1.0
        elif 30 <= h <= 500:
            size_score = 0.7
        else:
            size_score = 0.3

        center_y = y + h / 2
        if center_y > frame_height * 0.5:
            position_score = 1.0
        elif center_y > frame_height * 0.3:
            position_score = 0.8
        else:
            position_score = 0.5

        confidence = (
            0.4 * ratio_score +
            0.3 * size_score +
            0.3 * position_score
        )

        return min(confidence, 1.0)

    def _estimate_victim_distance(self, bbox: Tuple[int, int, int, int]) -> float:
        """Оценка расстояния до манекена (высота ~1.7м)."""
        x, y, w, h = bbox
        real_height = 1.7
        focal_length = 640

        if h == 0:
            return float('inf')

        distance = (real_height * focal_length) / h
        return distance

    # ==========================================
    # ДЕТЕКТИРОВАНИЕ ЗАВАЛОВ (БЫСТРОЕ, БЕЗ GLCM)
    # ==========================================
    # def _detect_collapse_fast(self, frame: np.ndarray) -> List[dict]:
    #     """
    #     Легковесное детектирование завалов через анализ плотности границ.
    #     Завалы имеют высокую плотность хаотичных краев по сравнению с полом или стенами.
    #     Работает только на стандартном OpenCV, потребляет минимум RAM.
    #     """
    #     collapses = []
        
    #     # Масштабируем для скорости
    #     scale = 0.5
    #     h_orig, w_orig = frame.shape[:2]
    #     small = cv2.resize(frame, (int(w_orig*scale), int(h_orig*scale)), 
    #                        interpolation=cv2.INTER_AREA)
        
    #     gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    #     gray = cv2.GaussianBlur(gray, (3, 3), 0)
        
    #     # Детектор границ Canny
    #     edges = cv2.Canny(gray, 50, 150)
        
    #     sh, sw = edges.shape
    #     ws = self.collapse_window_size
    #     step = ws // 2  # 50% перекрытие
        
    #     # Карта плотности границ
    #     density_map = np.zeros((sh, sw), dtype=np.float32)
        
    #     for y in range(0, sh - ws + 1, step):
    #         for x in range(0, sw - ws + 1, step):
    #             roi = edges[y:y+ws, x:x+ws]
    #             # Плотность = доля пикселей границ в окне
    #             density = np.countNonZero(roi) / (ws * ws)
    #             density_map[y:y+ws, x:x+ws] = density
        
    #     # Пороговая маска завала
    #     collapse_mask = (density_map > self.collapse_edge_density_threshold).astype(np.uint8)
        
    #     # Морфологические операции для объединения близких зон
    #     kernel = np.ones((9, 9), np.uint8)
    #     collapse_mask = cv2.morphologyEx(collapse_mask, cv2.MORPH_CLOSE, kernel)
    #     collapse_mask = cv2.morphologyEx(collapse_mask, cv2.MORPH_OPEN, kernel)
        
    #     # Поиск контуров
    #     contours, _ = cv2.findContours(collapse_mask, cv2.RETR_EXTERNAL, 
    #                                    cv2.CHAIN_APPROX_SIMPLE)
        
    #     for cnt in contours:
    #         area = cv2.contourArea(cnt)
    #         if area < self.min_collapse_area:
    #             continue
            
    #         x, y, w, h = cv2.boundingRect(cnt)
            
    #         # Масштабируем координаты обратно
    #         bbox = (int(x/scale), int(y/scale), int(w/scale), int(h/scale))
            
    #         # Расчет уверенности на основе средней плотности
    #         roi_mask = np.zeros((sh, sw), dtype=np.uint8)
    #         cv2.drawContours(roi_mask, [cnt], -1, 255, -1)
    #         valid = roi_mask > 0
            
    #         avg_density = np.mean(density_map[valid]) if np.any(valid) else 0
            
    #         # Уверенность растет с плотностью границ и размером
    #         conf = min(avg_density / 0.3, 1.0) * 0.7 + \
    #                min(area / 10000, 1.0) * 0.3
            
    #         collapses.append({
    #             'type': LandmarkType.COLLAPSE,
    #             'bbox': bbox,
    #             'confidence': conf,
    #             'distance': self._estimate_distance(bbox),
    #             'area_pixels': int(area / (scale * scale))
    #         })
            
    #     return collapses
