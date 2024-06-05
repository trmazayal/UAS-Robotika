import math

from mapping.bfs.Bfs import BFS
from mapping.constants import *


class BFSToReachDestination():
    def __init__(self, destination_actual_coordinate):
        self.canvas = None
        self.numpyMap = None
        self.arrived = False
        self.destination_actual_coordinate = destination_actual_coordinate
        self.bfs = BFS(self.stoppingCondition, lambda x, y, value: value >= PATH_OBSTACLE_TRESHOLD)

    def get_destination(self):
        return self.bfs.get_destination()

    def overall_destinations(self):
        return self.bfs.overall_destinations()

    def tick_to_check_if_need_replan(self, robot_pose):
        dx, dy = self.destination_actual_coordinate[0] - robot_pose.x, self.destination_actual_coordinate[1] - robot_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        if distance < 0.3:
            self.arrived = True

    def stoppingCondition(self, node_info):
        x = node_info.x
        y = node_info.y
        return self.destination_px_coord[0] == x and self.destination_px_coord[1] == y

    def move_on_to_next_destination(self):
        return self.bfs.move_on_to_next_destination()

    def set_map(self, numpyMap, currentActualPos):
        if self.arrived:
            return False
        self.numpyMap = numpyMap
        self.destination_px_coord = numpyMap.actual_to_px(self.destination_actual_coordinate)
        self.canvas = numpyMap.canvas
        return self.bfs.set_map(numpyMap, currentActualPos)

