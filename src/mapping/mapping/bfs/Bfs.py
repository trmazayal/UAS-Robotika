import collections

from mapping.constants import *
from mapping.bfs.Info import InfoNode
from mapping.bfs.QueueBfs import QueueBfs

class BFS():
    def __init__(self, stopping_condition, wall_condition):
        self.numpyMap = None
        self.currentActualPos = None
        self.routes = collections.deque()
        self.shortest_maps = []
        self.wall_condition = wall_condition
        self.stopping_condition = stopping_condition

    def is_diagonal(self,direction):
        directions = [NE, SE, SW, NW]
        return direction in directions

    def get_direction_x(self, direction):
        if direction == West or direction == NW or direction == SW:
            return -1
        if direction == East or direction == NE or direction == SE:
            return 1
        return 0

    def get_direction_y(self, direction):
        if direction == North or direction == NW or direction == NE:
            return -1
        if direction == South or direction == SW or direction == SE:
            return 1
        return 0

    def set_map(self, numpyMap, currentActualPos):
        self.routes.clear()
        self.numpyMap = numpyMap
        self.currentActualPos = currentActualPos
        self.currentPos = numpyMap.actual_to_px(self.currentActualPos)
        self.initialPositionIsWall = self.wall_condition(self.currentPos[0], self.currentPos[1],
                                                         numpyMap.canvas[self.currentPos[1]][self.currentPos[0]])
        self.shortest_maps = []
        for y in range(self.numpyMap.px_height):
            row = []
            for x in range(self.numpyMap.px_width):
                row.append(InfoNode(x, y))
            self.shortest_maps.append(row)
        final_node = self.bfs_find_route(self.stopping_condition)
        if final_node is None:
            return False
        self.final_node = final_node
        origin_coord = None
        for pixel_coord in self.backtrack_routes(final_node):
            act_x, act_y = self.numpyMap.px_to_actual_rect(pixel_coord).mid
            self.routes.appendleft((act_x, act_y, pixel_coord))
            origin_coord = pixel_coord
        self.origin_coord = origin_coord
        return True

    def backtrack_routes(self, final_node):
        if final_node is None:
            return
        curr_node = final_node
        last_direction = False
        while True:
            is_origin_point = (curr_node.shortest_distance == 0)
            if is_origin_point:
                yield curr_node.x, curr_node.y
                return
            if curr_node.direction_to_here != last_direction:
                yield curr_node.x, curr_node.y
                last_direction = curr_node.direction_to_here
            backtrack_direction = curr_node.direction_to_source
            dx, dy = self.get_direction_x(backtrack_direction), self.get_direction_y(backtrack_direction)
            curr_node = self.shortest_maps[curr_node.y + dy][curr_node.x + dx]

    def bfs_find_route(self, stopping_condition, empty_value=None):
        numpyMap = self.numpyMap
        self.routes.clear()
        if numpyMap is None:
            return
        queue = QueueBfs()
        initial_origin = self.shortest_maps[self.currentPos[1]][self.currentPos[0]]
        direction = South
        queue.push(direction, initial_origin)
        queue.new_distance(initializeOnly=True)
        wall_in_position = self.initialPositionIsWall

        while True:
            if queue.is_current_distance_empty():
                queue.new_distance()
                queue.to_next_distance()
            curr_node = queue.pop_item_at_current_distance(direction)
            if curr_node is None:
                return empty_value
            node_info = curr_node[0]
            direction = curr_node[1]
            distance = queue.currentDistance + (0.5 if self.is_diagonal(direction) else 0)
            if node_info.shortest_distance <= distance:
                continue
            if wall_in_position:
                wall_in_position = self.wall_condition(*node_info.xy, self.numpyMap.get_px(node_info.xy))
            node_info.set_shortest_distance(distance, direction)
            if stopping_condition(node_info):
                return node_info
            for next_dir in range(TOTAL_DIRECTION):
                dx, dy = self.get_direction_x(next_dir), self.get_direction_y(next_dir)
                x, y = node_info.x + dx, node_info.y + dy

                if not (0 <= x < self.numpyMap.px_width) or not (0 <= y < self.numpyMap.px_height):
                    continue
                if self.wall_condition(x, y, self.numpyMap.canvas[y][x]) and not wall_in_position:
                    continue
                next_node_info = self.shortest_maps[y][x]
                queue.push(next_dir, next_node_info)

    def overall_destinations(self):
        return self.routes

    def tick_to_check_if_need_replan(self, _):
        pass

    def get_destination(self):
        if len(self.routes) == 0:
            return self.move_on_to_next_destination()
        return self.routes[0]

    def move_on_to_next_destination(self):
        if len(self.routes) == 0:
            return None
        self.routes.popleft()
        return self.get_destination()