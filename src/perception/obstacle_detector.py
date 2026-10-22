import numpy as np
from src.utils.config import (
    FRONT_CENTER, LEFT_CENTER, RIGHT_CENTER,
    FRONT_ANGLE_RANGE, SIDE_ANGLE_RANGE,
    STOP_DISTANCE_MM, WALL_TOO_CLOSE_MM, WALL_TOO_FAR_MM
)

class ObstacleDetector:
    @staticmethod
    def get_zone_distances(scan_data, center_angle, angle_range):
        if not scan_data or len(scan_data) == 0:
            return float('inf')
        scan_array = np.array(scan_data)
        angles = scan_array[:, 1]
        distances = scan_array[:, 2]
        diff = np.abs(angles - center_angle)
        diff = np.minimum(diff, 360 - diff)
        mask = diff <= angle_range
        zone_distances = distances[mask]
        zone_distances = zone_distances[zone_distances > 10]
        if len(zone_distances) == 0:
            return float('inf')
        return np.min(zone_distances)

    @staticmethod
    def get_zone_distances_with_count(scan_data, center_angle, angle_range):
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

    def check_front_obstacle(self, scan_data):
        return self.get_zone_distances(scan_data, FRONT_CENTER, FRONT_ANGLE_RANGE)

    def check_left_obstacle(self, scan_data):
        return self.get_zone_distances(scan_data, LEFT_CENTER, SIDE_ANGLE_RANGE)

    def check_right_obstacle(self, scan_data):
        return self.get_zone_distances(scan_data, RIGHT_CENTER, SIDE_ANGLE_RANGE)

    def detect_wall_side(self, scan, extended_range=False):
        if scan is None or len(scan) == 0:
            return None, float('inf'), 0
        
        min_dist = 150 if extended_range else WALL_TOO_CLOSE_MM
        max_dist = 700 if extended_range else WALL_TOO_FAR_MM
        min_points = 5 if extended_range else 10
        
        left_dist, left_count = self.get_zone_distances_with_count(scan, LEFT_CENTER, SIDE_ANGLE_RANGE) # LEFT_CENTER = 90
        right_dist, right_count = self.get_zone_distances_with_count(scan, RIGHT_CENTER, SIDE_ANGLE_RANGE) # RIGHT_CENTER = 270
        
        left_in_range = min_dist < left_dist < max_dist and left_count >= min_points
        right_in_range = min_dist < right_dist < max_dist and right_count >= min_points
        
        # Return physical sides consistently with the configured lidar frame.
        if left_in_range and (right_dist > left_dist or not right_in_range):
            return 'left', left_dist, left_count
        elif right_in_range and (left_dist > right_dist or not left_in_range):
            return 'right', right_dist, right_count
        return None, float('inf'), 0

    def find_nearest_wall_direction(self, scan_data):
        if not scan_data or len(scan_data) == 0:
            return None, float('inf')
        scan_array = np.array(scan_data)
        angles = scan_array[:, 1]
        distances = scan_array[:, 2]
        valid_mask = distances > 10
        valid_angles = angles[valid_mask]
        valid_distances = distances[valid_mask]
        if len(valid_distances) == 0:
            return None, float('inf')
        min_idx = np.argmin(valid_distances)
        nearest_angle = valid_angles[min_idx]
        nearest_dist = valid_distances[min_idx]
        if 135 < nearest_angle < 225:
            return 'front', nearest_dist
        elif 45 < nearest_angle < 135:
            return 'left', nearest_dist
        elif 225 < nearest_angle < 315:
            return 'right', nearest_dist
        else:
            return 'back', nearest_dist