import time
from src.utils.config import (
    STATE_MOVING, STATE_TURNING, STATE_WALL_FOLLOWING,
    STATE_SEARCHING_WALL, STATE_SMALL_TURN, STATE_PAUSED
)

class RobotStateMachine:
    def __init__(self):
        self.state = STATE_MOVING
        self.is_paused = False
        self.wall_side = None
        self.turn_start_time = 0
        self.small_turn_start_time = 0
        self.small_turn_direction = None
        self.next_state_after_small_turn = STATE_MOVING
        self.obstacle_side = None

    def set_state(self, new_state):
        self.state = new_state

    def get_state(self):
        return self.state

    def start_small_turn(self, direction, next_state=STATE_MOVING):
        self.small_turn_direction = direction
        self.small_turn_start_time = time.time()
        self.next_state_after_small_turn = next_state

    def get_small_turn_duration(self):
        return time.time() - self.small_turn_start_time

    def start_turning(self, wall_side, obstacle_side=None):
        self.wall_side = wall_side
        self.obstacle_side = obstacle_side
        self.turn_start_time = time.time()