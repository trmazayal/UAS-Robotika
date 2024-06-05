from mapping.constants import *

class InfoNode():
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.shortest_distance = float('inf')
        self.direction_to_here = None
        self.shortest_distance_initialized = False

    @property
    def direction_to_source(self):
        if self.direction_to_here == North:
            return South
        if self.direction_to_here == NE:
            return SW
        if self.direction_to_here == East:
            return West
        if self.direction_to_here == SE:
            return NW
        if self.direction_to_here == South:
            return North
        if self.direction_to_here == SW:
            return NE
        if self.direction_to_here == West:
            return East
        if self.direction_to_here == NW:
            return SE

    def set_shortest_distance(self, shortest_distance, direction_to_here):
        assert shortest_distance < self.shortest_distance
        self.shortest_distance = shortest_distance
        self.direction_to_here = direction_to_here
        self.shortest_distance_initialized = True

    @property
    def xy(self):
        return self.x, self.y