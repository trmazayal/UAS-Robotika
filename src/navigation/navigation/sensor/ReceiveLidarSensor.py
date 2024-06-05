import math

import numpy as np

class ReceiveLidarSensor:
    def __init__(self, vision_angle):
        assert vision_angle >= 0
        self.vision_angle = vision_angle
        self.obstacle_distances =np.array([])
        self.obstacle_angles =np.array([])

    def set_obstacle_distances(self, distances):
        self.obstacle_distances = distances

    def set_obstacle_angles(self, angles):
        self.obstacle_angles = angles

    def is_vision_blocked(self, target_point, robot_pose):
        assert len(self.obstacle_distances) == len(self.obstacle_angles)
        if len(self.obstacle_distances) == 0:
            return True
        dx, dy = target_point[0] - robot_pose.x, target_point[1] - robot_pose.y
        length = (dx**2 + dy**2)**0.5

        angle = math.atan2(dy, dx)
        angle -= robot_pose.theta
        if abs(angle) > math.pi:
            # keep the angle range between -pi and pi
            angle %= 2 * math.pi
        if angle > math.pi:
            angle -= 2 * math.pi
        closest_angle = np.searchsorted(self.obstacle_angles, angle)

        i = closest_angle
        while i < len(self.obstacle_angles) and abs(self.obstacle_angles[i] - angle) < self.vision_angle:
            if self.obstacle_distances[i] < length:
                return True
            i += 1
        if i == len(self.obstacle_angles):
            # check the first element
            return True
        i = closest_angle
        while i >= 0 and abs(self.obstacle_angles[i] - angle) < self.vision_angle:
            if self.obstacle_distances[i] < length:
                return True
            i -= 1
        if i <= -1:
            # check the last element
            return True
        return False
