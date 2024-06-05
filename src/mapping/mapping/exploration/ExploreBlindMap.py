from mapping.constants import *
from mapping.bfs.Bfs import BFS

class ExploreBlindMap():
    def __init__(self):
        self.bfs = BFS(self.stoppingCondition, lambda x, y, value: value >= PATH_OBSTACLE_TRESHOLD)
        self.canvas = None
        self.numpyMap = None
        self.currentActualPos = None

    def get_destination(self):
        return self.bfs.get_destination()

    def overall_destinations(self):
        return self.bfs.overall_destinations()

    def tick_to_check_if_need_replan(self, robot_pose):
        temp = self.bfs.overall_destinations()
        if temp is None or len(temp) == 0:
            return
        _, _, final_dest = temp[-1]
        if self.numpyMap.canvas[final_dest[1]][final_dest[0]] and robot_pose is not None:
            self.bfs.set_map(self.numpyMap, (robot_pose.x, robot_pose.y))

    def set_map(self, numpyMap, currentActualPos):
        self.numpyMap = numpyMap
        self.canvas = numpyMap.canvas
        self.currentActualPos = currentActualPos
        return self.bfs.set_map(numpyMap, currentActualPos)

    def redo_bfs(self):
        self.bfs.set_map(self.numpyMap, self.currentActualPos)

    def stoppingCondition(self, node_info):
        x, y = node_info.x, node_info.y
        return self.canvas[y][x] == PATH_UNKNOWN_VALUE

    def move_on_to_next_destination(self):
        return self.bfs.move_on_to_next_destination()
