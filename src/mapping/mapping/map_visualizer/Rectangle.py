import numpy as np

class Rectangle:
    def __init__(self, min_x, min_y, max_x, max_y):
        self.max_x, self.max_y = max_x, max_y
        self.min_x, self.min_y = min_x, min_y
        assert self.min_x <= self.max_x
        assert self.min_y <= self.max_y

    @property
    def mid(self):
        mid_x = (self.min_x + self.max_x) / 2
        mid_y = (self.min_y + self.max_y) / 2
        return mid_x, mid_y

    @property
    def start(self):
        x = self.min_x
        y = self.min_y
        return x, y

    @property
    def end(self):
        x = self.max_x
        y = self.max_y
        return x, y

    def slice_map(self, np_canvas_array) -> np.ndarray:
        result = np_canvas_array[self.min_y:self.max_y, self.min_x:self.max_x]
        return result