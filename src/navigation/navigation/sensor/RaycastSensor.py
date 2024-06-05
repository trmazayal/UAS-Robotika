import math
import numpy as np

from navigation.constants import *

class RaycastSensor:
    def __init__(self, raycast_length, robot_radius=ROBOT_RADIUS):
        assert robot_radius >= 0
        self._distances, self._angles, self._x, self._y = np.array([]), np.array([]), np.array([]), np.array([])
        self._dirty_bit = False
        self._closest_left = None
        self._closest_right = None
        self.robot_radius = robot_radius
        self.raycast_length = raycast_length

    @property
    def obstacle_hits(self):
        return self._distances, self._angles

    @obstacle_hits.setter
    def obstacle_hits(self, value):
        self._distances: np.ndarray = value[0]
        self._angles: np.ndarray = value[1]
        in_front = (math.radians(-90) < self._angles) & (self._angles < math.radians(90))
        self._distances, self._angles = self._distances[in_front], self._angles[in_front]
        argsort = np.flip(self._distances.argsort())
        self._distances, self._angles = self._distances[argsort], self._angles[argsort]
        self._dirty_bit = True

    def _update_xy(self):
        if not self._dirty_bit:
            return
        self._x, self._y = self._distances * np.cos(np.pi/2 + self._angles), self._distances * np.sin(np.pi/2 + self._angles)
        coordinates = np.stack((self._x, self._y), axis=1)
        distances = np.linalg.norm(coordinates, axis=1)

        closest_left_index = np.where((self._x > -self.robot_radius) & (self._x <= 0), distances,
                                      np.inf).argmin()
        closest_right_index = np.where((self._x > 0) & (self._x < self.robot_radius), distances,
                                       np.inf).argmin()
        self._closest_left = self._x[closest_left_index], self._y[closest_left_index]
        self._closest_right = self._x[closest_right_index], self._y[closest_right_index]
        self._closest_left_index = closest_left_index
        self.closest_right_index = closest_right_index
        assert self._closest_left[1] >= 0
        assert self._closest_right[1] >= 0

    @property
    def closest_left(self):
        self._update_xy()
        if self._closest_left is None or self._closest_left[1] >= self.raycast_length:
            return None
        return self._closest_left

    @property
    def closest_right(self):
        self._update_xy()
        if self._closest_right is None or self._closest_right[1] >= self.raycast_length:
            return None
        return self._closest_right

    @property
    def closest_overall(self):
        left = self.closest_left
        right = self.closest_right
        if left is None:
            return right
        if right is None:
            return left
        if left[1] >= right[1]:
            return right
        return left