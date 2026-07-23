import time
from src.utils.config import (
    STATE_MOVING, STATE_TURNING, STATE_WALL_FOLLOWING,
    STATE_SEARCHING_WALL, STATE_SMALL_TURN, STATE_PAUSED,
    WALL_TOO_CLOSE_MM, WALL_TOO_FAR_MM, STOP_DISTANCE_MM,
    BASE_SPEED, ROTATE_SPEED, CORRECT_SPEED, WALL_SEARCH_TIMEOUT,
    SMALL_TURN_DURATION
)

class RobotController:
    def __init__(self, motor_controller, obstacle_detector, state_machine):
        self.motor = motor_controller
        self.detector = obstacle_detector
        self.state_machine = state_machine
        self.last_obstacle_dist = None
        self.obstacle_stuck_time = 0
        self.wall_following_start_time = 0
        self.cooldown_duration = 1.5

    def handle_moving_state(self, scan):
        if scan is None or len(scan) == 0:
            self.motor.move_forward(BASE_SPEED)
            return 'State: MOVING (no lidar)', 'yellow', 'Lidar offline'

        front_dist = self.detector.check_front_obstacle(scan)
        
        if front_dist < STOP_DISTANCE_MM:
            self.state_machine.start_turning(None)
            self.state_machine.set_state(STATE_TURNING)
            self.last_obstacle_dist = None
            return 'State: TURNING (obstacle)', 'red', f'Obstacle: {front_dist:.0f}mm'

        wall_side, wall_dist, wall_count = self.detector.detect_wall_side(scan, extended_range=True)
        if wall_side is not None:
            # wall_side теперь означает физическую сторону стены
            self.state_machine.wall_side = wall_side
            self.state_machine.set_state(STATE_WALL_FOLLOWING)
            self.wall_following_start_time = time.time()
            return f'State: WALL FOLLOWING ({wall_side})', 'green', f'Wall: {wall_dist:.0f}mm'

        if front_dist < WALL_TOO_FAR_MM:
            self.state_machine.start_turning(None)
            self.state_machine.set_state(STATE_TURNING)
            self.last_obstacle_dist = None
            return 'State: TURNING (approach)', 'cyan', f'Approaching wall: {front_dist:.0f}mm'

        self.motor.move_forward(BASE_SPEED)
        return 'State: MOVING', 'yellow', 'Searching for wall'

    def handle_wall_following_state(self, scan):
        if scan is None or len(scan) == 0:
            self.motor.move_forward(BASE_SPEED)
            return f'State: WALL FOLLOWING ({self.state_machine.wall_side})', 'green', 'No lidar'

        elapsed_since_start = time.time() - self.wall_following_start_time
        in_cooldown = elapsed_since_start < self.cooldown_duration

        if in_cooldown:
            self.motor.move_forward(BASE_SPEED)
            return f'State: WALL FOLLOWING ({self.state_machine.wall_side}) [cooldown]', 'green', f'{self.cooldown_duration - elapsed_since_start:.1f}s'

        front_dist = self.detector.check_front_obstacle(scan)
        
        if front_dist < STOP_DISTANCE_MM:
            self.state_machine.start_turning(self.state_machine.wall_side)
            self.state_machine.set_state(STATE_TURNING)
            self.last_obstacle_dist = None
            return 'State: TURNING', 'red', f'Obstacle: {front_dist:.0f}mm'

        wall_side, wall_dist, wall_count = self.detector.detect_wall_side(scan, extended_range=True)
        
        if wall_side is None:
            self.state_machine.wall_side = None
            self.state_machine.set_state(STATE_SEARCHING_WALL)
            return 'State: SEARCHING WALL', 'orange', 'Lost wall'

        if wall_side != self.state_machine.wall_side:
            self.state_machine.wall_side = wall_side

        # ЛОГИКА КОРРЕКЦИИ: теперь wall_side означает физическую сторону
        if wall_side == 'left': # Стена физически слева
            if wall_dist < WALL_TOO_CLOSE_MM: # Слишком близко к левой стене
                self.motor.move_forward_with_arc('right', BASE_SPEED, CORRECT_SPEED) # Дуга вправо, чтобы отъехать
                return 'State: WALL FOLLOWING (left, correcting right)', 'cyan', f'Too close: {wall_dist:.0f}mm'
            elif wall_dist > WALL_TOO_FAR_MM: # Слишком далеко от левой стены
                self.motor.move_forward_with_arc('left', BASE_SPEED, CORRECT_SPEED)  # Дуга влево, чтобы подъехать
                return 'State: WALL FOLLOWING (left, correcting left)', 'cyan', f'Too far: {wall_dist:.0f}mm'
            else:
                self.motor.move_forward(BASE_SPEED)
                return 'State: WALL FOLLOWING (left)', 'green', f'Distance: {wall_dist:.0f}mm'
        else: # wall_side == 'right'. Стена физически справа
            if wall_dist < WALL_TOO_CLOSE_MM: # Слишком близко к правой стене
                self.motor.move_forward_with_arc('left', BASE_SPEED, CORRECT_SPEED)  # Дуга влево, чтобы отъехать
                return 'State: WALL FOLLOWING (right, correcting left)', 'cyan', f'Too close: {wall_dist:.0f}mm'
            elif wall_dist > WALL_TOO_FAR_MM: # Слишком далеко от правой стены
                self.motor.move_forward_with_arc('right', BASE_SPEED, CORRECT_SPEED) # Дуга вправо, чтобы подъехать
                return 'State: WALL FOLLOWING (right, correcting right)', 'cyan', f'Too far: {wall_dist:.0f}mm'
            else:
                self.motor.move_forward(BASE_SPEED)
                return 'State: WALL FOLLOWING (right)', 'green', f'Distance: {wall_dist:.0f}mm'

    def handle_small_turn_state(self, scan):
        duration = self.state_machine.get_small_turn_duration()
        direction = self.state_machine.small_turn_direction

        if direction == 'left':
            self.motor.rotate_left(CORRECT_SPEED)
        elif direction == 'right':
            self.motor.rotate_right(CORRECT_SPEED)

        if duration > SMALL_TURN_DURATION:
            self.motor.stop_all()
            next_state = self.state_machine.next_state_after_small_turn
            self.state_machine.set_state(next_state)
            if next_state == STATE_WALL_FOLLOWING:
                self.wall_following_start_time = time.time()
            return f'State: {next_state.upper()}', 'green', 'Small turn complete'

        return f'State: SMALL TURN ({direction})', 'cyan', f'Duration: {duration:.1f}s'

    def handle_turning_state(self, scan):
        current_time = time.time()
        elapsed = current_time - self.state_machine.turn_start_time
        wall_side = self.state_machine.wall_side # Теперь это физическая сторона стены

        if scan is not None and len(scan) > 0:
            front_dist = self.detector.check_front_obstacle(scan)
            
            if self.last_obstacle_dist is not None:
                if abs(front_dist - self.last_obstacle_dist) < 10:
                    if self.obstacle_stuck_time == 0:
                        self.obstacle_stuck_time = current_time
                    elif current_time - self.obstacle_stuck_time > 2.0:
                        self.motor.stop_all()
                        self.state_machine.wall_side = None
                        self.state_machine.set_state(STATE_SEARCHING_WALL)
                        self.last_obstacle_dist = None
                        self.obstacle_stuck_time = 0
                        return 'State: SEARCHING WALL', 'orange', 'Turn stuck'
                else:
                    self.obstacle_stuck_time = 0
            
            self.last_obstacle_dist = front_dist

            new_wall_side, new_wall_dist, new_wall_count = self.detector.detect_wall_side(scan)
            
            # ВЕРНУТО СТРОГОЕ УСЛОВИЕ
            front_clear = front_dist > WALL_TOO_FAR_MM
            
            # УСЛОВИЕ ВЫХОДА: Найдена боковая стена в допустимом диапазоне И фронт свободен
            wall_in_acceptable_range = WALL_TOO_CLOSE_MM < new_wall_dist < WALL_TOO_FAR_MM

            # Основное условие выхода: стена сбоку найдена и в нужном диапазоне И фронт свободен
            if new_wall_side is not None and wall_in_acceptable_range and front_clear:
                self.motor.stop_all()
                self.state_machine.wall_side = new_wall_side
                self.state_machine.set_state(STATE_WALL_FOLLOWING)
                self.wall_following_start_time = time.time()
                self.last_obstacle_dist = None
                self.obstacle_stuck_time = 0
                return f'State: WALL FOLLOWING ({new_wall_side})', 'green', f'Found: {new_wall_dist:.0f}mm, front: {front_dist:.0f}mm'

        # ПОВОРОТ ОТ СТЕНЫ: теперь wall_side означает физическую сторону
        # Чтобы отъехать от стены, нужно повернуть ПРОТИВОПОЛОЖНО от её стороны
        if wall_side == 'left': # Стена физически слева -> поворачиваем направо, чтобы отъехать
            self.motor.rotate_right(ROTATE_SPEED)
        elif wall_side == 'right': # Стена физически справа -> поворачиваем налево, чтобы отъехать
            self.motor.rotate_left(ROTATE_SPEED)
        else:
            self.motor.rotate_right(ROTATE_SPEED) # По умолчанию

        if elapsed > WALL_SEARCH_TIMEOUT:
            self.motor.stop_all()
            self.state_machine.wall_side = None
            self.state_machine.set_state(STATE_SEARCHING_WALL)
            self.last_obstacle_dist = None
            self.obstacle_stuck_time = 0
            return 'State: SEARCHING WALL', 'orange', 'Turn timeout'

        return f'State: TURNING', 'red', f'Elapsed: {elapsed:.1f}s, front: {self.last_obstacle_dist}'

    def handle_searching_wall_state(self, scan):
        if scan is None or len(scan) == 0:
            self.motor.move_forward(BASE_SPEED)
            return 'State: SEARCHING WALL', 'orange', 'No lidar'

        direction, nearest_dist = self.detector.find_nearest_wall_direction(scan)

        if direction == 'front' and nearest_dist < WALL_TOO_FAR_MM:
            self.state_machine.start_turning(None)
            self.state_machine.set_state(STATE_TURNING)
            self.last_obstacle_dist = None
            return 'State: TURNING (approach wall)', 'cyan', f'Front wall: {nearest_dist:.0f}mm'

        wall_side, wall_dist, wall_count = self.detector.detect_wall_side(scan, extended_range=True)
        if wall_side is not None:
            # wall_side теперь физическая сторона
            self.motor.stop_all()
            self.state_machine.wall_side = wall_side
            # Выбираем направление поворота для выравнивания
            # Если стена физически слева ('left'), выравниваемся поворотом вправо ('right')
            # Если стена физически справа ('right'), выравниваемся поворотом влево ('left')
            align_direction = 'right' if wall_side == 'left' else 'left'
            self.state_machine.start_small_turn(align_direction, STATE_MOVING)
            self.state_machine.set_state(STATE_SMALL_TURN)
            return f'State: SMALL TURN (align to {wall_side})', 'green', f'Found wall: {wall_dist:.0f}mm'

        front_dist = self.detector.check_front_obstacle(scan)
        if front_dist < STOP_DISTANCE_MM:
            self.motor.stop_all()
            self.state_machine.start_small_turn('right', STATE_SEARCHING_WALL)
            self.state_machine.set_state(STATE_SMALL_TURN)
            return 'State: SMALL TURN (avoid)', 'orange', f'Obstacle: {front_dist:.0f}mm'

        self.motor.move_forward(BASE_SPEED)
        return 'State: SEARCHING WALL', 'orange', 'Moving forward'