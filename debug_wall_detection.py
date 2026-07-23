#!/usr/bin/env python3
import sys
import os
import time
import numpy as np

# Добавляем путь к src
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Импортируем нужные классы
from src.hardware.motor_controller import MotorController
from src.hardware.lidar_controller import LidarController
from src.perception.obstacle_detector import ObstacleDetector

# Импортируем конфигурацию
from src.utils.config import (
    LEFT_CENTER, RIGHT_CENTER, SIDE_ANGLE_RANGE,
    BASE_SPEED, CORRECT_SPEED, ROTATE_SPEED,
    WALL_TOO_CLOSE_MM, WALL_TOO_FAR_MM
)

def get_zone_distances_with_count(scan_data, center_angle, angle_range):
    """Версия функции из ObstacleDetector для использования в тесте."""
    if not scan_data or len(scan_data) == 0:
        return float('inf'), 0
    scan_array = np.array(scan_data)
    angles = scan_array[:, 1]
    distances = scan_array[:, 2]
    diff = np.abs(angles - center_angle)
    diff = np.minimum(diff, 360 - diff)
    mask = diff <= angle_range
    zone_distances = distances[mask]
    valid = zone_distances[zone_distances > 10]
    count = len(valid)
    if count == 0:
        return float('inf'), 0
    return np.min(valid), count

def detect_wall_side(scan, extended_range=False):
    """Версия функции из ObstacleDetector для использования в тесте."""
    if scan is None or len(scan) == 0:
        return None, float('inf'), 0
    
    min_dist = 150 if extended_range else WALL_TOO_CLOSE_MM
    max_dist = 700 if extended_range else WALL_TOO_FAR_MM
    min_points = 5 if extended_range else 10
    
    left_dist, left_count = get_zone_distances_with_count(scan, LEFT_CENTER, SIDE_ANGLE_RANGE) # LEFT_CENTER = 270
    right_dist, right_count = get_zone_distances_with_count(scan, RIGHT_CENTER, SIDE_ANGLE_RANGE) # RIGHT_CENTER = 90
    
    left_in_range = min_dist < left_dist < max_dist and left_count >= min_points
    right_in_range = min_dist < right_dist < max_dist and right_count >= min_points
    
    print(f"  Debug: Left zone (270°) dist: {left_dist:.2f}, count: {left_count}, in_range: {left_in_range}")
    print(f"  Debug: Right zone (90°) dist: {right_dist:.2f}, count: {right_count}, in_range: {right_in_range}")

    if left_in_range and (right_dist > left_dist or not right_in_range):
        return 'left', left_dist, left_count
    elif right_in_range and (left_dist > right_dist or not left_in_range):
        return 'right', right_dist, right_count
    return None, float('inf'), 0


def main():
    print("=== Тестовая Программа для Определения Стороны Стены и Направления Поворота ===")
    print("Убедитесь, что робот находится у стены СЛЕВА от себя.")
    print("После запуска наблюдайте за поведением робота и отвечайте на вопросы в консоли.")
    input("Нажмите Enter для продолжения...")

    try:
        # Инициализация
        print("Инициализация оборудования...")
        motor_ctrl = MotorController()
        lidar_ctrl = LidarController()
        detector = ObstacleDetector() # Хотя мы используем свою версию detect_wall_side
        print("Оборудование инициализировано.")

        # Небольшая задержка, чтобы лидар "разогрелся"
        print("Ожидание готовности лидара (2 секунды)...")
        time.sleep(2.0)

        # Получение скана
        print("Получение скана лидара...")
        scan_attempts = 0
        max_attempts = 10
        scan = None
        while scan is None and scan_attempts < max_attempts:
            scan = lidar_ctrl.get_fresh_scan(timeout=0.5) # Уменьшенный таймаут для одной попытки
            scan_attempts += 1
            if scan is None:
                 print(f"  Попытка {scan_attempts}: Скан не получен. Ждем немного...")
                 time.sleep(0.1) # Пауза между попытками

        if scan is None or len(scan) == 0:
            print("Ошибка: Не удалось получить скан лидара после нескольких попыток.")
            return
        print("Скан получен.")

        # Определение стороны стены
        wall_side, wall_dist, wall_count = detect_wall_side(scan, extended_range=True)
        print(f"\nРЕЗУЛЬТАТ: Определенная сторона стены: {wall_side} (расстояние: {wall_dist:.2f} мм, точек: {wall_count})")
        print(f"Ожидаемое значение: 'left' (так как стена физически слева)")
        print(f"Соответствует ли это ожиданию? (y/n): ", end="")
        response = input().lower()
        if response == 'n':
             print("ЛОГИКА ОПРЕДЕЛЕНИЯ СТОРОНЫ СТЕНЫ ТРЕБУЕТ КОРРЕКТИРОВКИ.")


        print("\n--- Тест move_forward_with_arc ---")
        print("1. Выполнение move_forward_with_arc('left', ...)")
        print("   Наблюдайте: робот поворачивает дугой ВЛЕВО или ВПРАВО?")
        motor_ctrl.move_forward_with_arc('left', BASE_SPEED, CORRECT_SPEED)
        time.sleep(1.0)
        motor_ctrl.stop_all()
        arc_left_response = input("   Введите 'left' если дуга влево, 'right' если дуга вправо: ").lower()
        print(f"   Записано: дуга '{arc_left_response}' для команды 'left'")
        time.sleep(1.0)

        print("\n2. Выполнение move_forward_with_arc('right', ...)")
        print("   Наблюдайте: робот поворачивает дугой ВЛЕВО или ВПРАВО?")
        motor_ctrl.move_forward_with_arc('right', BASE_SPEED, CORRECT_SPEED)
        time.sleep(1.0)
        motor_ctrl.stop_all()
        arc_right_response = input("   Введите 'left' если дуга влево, 'right' если дуга вправо: ").lower()
        print(f"   Записано: дуга '{arc_right_response}' для команды 'right'")
        time.sleep(1.0)

        print("\n--- Тест rotate_left / rotate_right ---")
        print("3. Выполнение rotate_left(...)")
        print("   Наблюдайте: робот поворачивается НАЛЕВО или НАПРАВО?")
        motor_ctrl.rotate_left(ROTATE_SPEED)
        time.sleep(0.5) # Поворот на 0.5 секунды
        motor_ctrl.stop_all()
        rot_left_response = input("   Введите 'left' если поворот налево, 'right' если направо: ").lower()
        print(f"   Записано: поворот '{rot_left_response}' для команды rotate_left")
        time.sleep(1.0)

        print("\n4. Выполнение rotate_right(...)")
        print("   Наблюдайте: робот поворачивается НАЛЕВО или НАПРАВО?")
        motor_ctrl.rotate_right(ROTATE_SPEED)
        time.sleep(0.5) # Поворот на 0.5 секунды
        motor_ctrl.stop_all()
        rot_right_response = input("   Введите 'left' если поворот налево, 'right' если направо: ").lower()
        print(f"   Записано: поворот '{rot_right_response}' для команды rotate_right")
        time.sleep(1.0)

        print("\n--- Результаты Теста ---")
        print(f"Определенная сторона стены: {wall_side}")
        print(f"Физическая сторона стены (по вашему мнению): left (если слева от робота)")
        print(f"Дуга 'left' -> фактически: {arc_left_response}")
        print(f"Дуга 'right' -> фактически: {arc_right_response}")
        print(f"Поворот rotate_left -> фактически: {rot_left_response}")
        print(f"Поворот rotate_right -> фактически: {rot_right_response}")

        print("\n--- Рекомендации для исправления ---")
        print("1. Если 'Определенная сторона стены' НЕ соответствует физической стороне,")
        print("   измените логику в функции detect_wall_side (или в месте её вызова в основном коде).")
        print("2. Если 'Дуга left/right' или 'Поворот left/right' НЕ соответствуют ожиданиям,")
        print("   измените логику в функциях move_forward_with_arc или rotate_left/right")
        print("   в файле src/hardware/motor_controller.py.")
        print("3. Обновите логику в src/control/robot_controller.py, чтобы она соответствовала")
        print("   исправленной интерпретации сторон и команд поворота.")


    except KeyboardInterrupt:
        print("\nТест прерван пользователем.")
    finally:
        print("Остановка моторов...")
        try:
            motor_ctrl.stop_all()
        except:
            pass
        print("Отключение лидара...")
        try:
            lidar_ctrl.shutdown()
        except:
            pass
        print("Тест завершен.")

if __name__ == "__main__":
    main()