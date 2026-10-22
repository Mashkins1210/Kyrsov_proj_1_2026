"""Incremental lidar localization, anchored by the operator at the start.

This bounds short-term encoder drift; it does not certify global map accuracy.
Rejected or ambiguous geometry stops navigation instead of inventing a pose.
"""
import math
import time
import numpy as np
from src.control.scan_registration import align_scans


def relative_pose(start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return (c*dx+s*dy, -s*dx+c*dy,
            math.atan2(math.sin(end[2]-start[2]), math.cos(end[2]-start[2])))


def compose_pose(start, delta):
    c, s = math.cos(start[2]), math.sin(start[2])
    angle = start[2]+delta[2]
    return (start[0]+c*delta[0]-s*delta[1], start[1]+s*delta[0]+c*delta[1],
            math.atan2(math.sin(angle), math.cos(angle)))


class ScanLocalization:
    def __init__(self, settings):
        self.settings = settings
        self.cloud = self.odom = self.pose = None
        self.stamp = None
        self.last_accepted = None
        self.last_result = None

    def points(self, scan):
        points = []
        for q, a, d in scan:
            if q > 0 and 200 < d < 5000:
                angle = math.radians(self.settings['lidar_angle_sign']*(a-self.settings['lidar_front_deg']))
                points.append((d/1000*math.cos(angle), d/1000*math.sin(angle)))
        points = np.asarray(points, dtype=float)
        if len(points) < 50:
            raise RuntimeError('Localization requires at least 50 lidar points')
        return points[::max(1, math.ceil(len(points)/240))]

    def update(self, scan, stamp, odom):
        now = time.monotonic()
        if self.cloud is None:
            self.cloud = self.points(scan)
            self.odom = self.pose = tuple(odom)
            self.stamp = stamp
            self.last_accepted = now
            return self.pose
        predicted = compose_pose(self.pose, relative_pose(self.odom, odom))
        if stamp != self.stamp:
            current = self.points(scan)
            guess = relative_pose(self.odom, odom)
            # The stationary encoder baseline prevents accumulated scan noise
            # from making a parked robot drift into unknown occupancy cells.
            if math.hypot(guess[0],guess[1]) < .001 and abs(guess[2]) < .001:
                self.cloud, self.odom, self.stamp = current, tuple(odom), stamp
                self.last_accepted = now
                self.last_result = {'accepted':True,'stationary_encoder_baseline':True}
                return self.pose
            result = align_scans(current, self.cloud, initial=guess)
            self.last_result = result
            self.stamp = stamp
            if result['accepted']:
                delta = (*result['translation_m'], result['yaw_rad'])
                if math.hypot(delta[0]-guess[0], delta[1]-guess[1]) > .06 or abs(delta[2]-guess[2]) > .15:
                    raise RuntimeError('Lidar and encoder motion disagree; localization stopped')
                self.pose = compose_pose(self.pose, delta)
                self.odom, self.cloud = tuple(odom), current
                self.last_accepted = now
                return self.pose
        if now-self.last_accepted > .8:
            raise RuntimeError('Lidar localization ambiguous or stale; navigation stopped')
        return predicted
