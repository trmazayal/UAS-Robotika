import cv2
import math
import numpy as np

from mapping.constants import *
from mapping.map_visualizer.Rectangle import Rectangle


class NumPyMap:
    def __init__(self, map_start = START_NAVIGATION, map_end = FINISH_NAVIGATION, resolution=RESOLUTION, show_image=True):
        self.robot_pos = None
        self.map_start = map_start
        self.min_x, self.min_y = map_start
        self.max_x, self.max_y = map_end
        self.resolution = resolution
        self.map_end = map_end

        self.act_width = self.max_x - self.min_x
        self.act_height = self.max_y - self.min_y
        self.px_width = math.ceil(self.act_width / resolution)
        self.px_height = math.ceil(self.act_height / resolution)
        self.px_anchor_x, self.px_anchor_y = round(-self.min_x / resolution), round(-self.min_y / resolution)

        self.canvas = np.full((self.px_height, self.px_width, 1), PATH_UNKNOWN_VALUE, dtype=np.uint8)

    def get_px(self, coordinate):
        return self.canvas[coordinate[1]][coordinate[0]]

    def actual_to_px(self, point):
        return (self.x_to_px(point[0]), self.y_to_px(point[1]))

    def px_to_actual_rect(self, px):
        px_x, px_y = px
        percentage_x = px_x / self.px_width
        percentage_x_end = (px_x + 1) / self.px_width

        percentage_y1, percentage_y2 = px_y / self.px_height, (px_y + 1) / self.px_height
        y1, y2 = self.max_y - self.act_height * percentage_y1, self.max_y - self.act_height * percentage_y2

        result = Rectangle(self.min_x + self.act_width * percentage_x, min(y1, y2),
                            self.min_x + self.act_width * percentage_x_end, max(y1, y2))

        return result

    def actual_rect_to_px_rect(self, actual_rect):
        x_start, y_start = self.actual_to_px(actual_rect.start)
        x_end, y_end = self.actual_to_px(actual_rect.end)

        if x_end < x_start:
            x_start, x_end = x_end, x_start
        if y_end < y_start:
            y_start, y_end = y_end, y_start

        result = Rectangle(x_start, y_start, x_end, y_end)
        return result

    def px_to_x(self, px):
        percentage = px / self.px_width
        result = self.min_x + self.act_width * percentage

        if result < self.min_x:
            return self.min_x

        if result > self.max_x:
            return self.max_x

        return result

    def px_to_y(self, py):
        percentage = 1 - py / self.px_height
        result = self.min_y + self.act_height * percentage
        if result < self.min_y:
            return self.min_y
        if result > self.max_y:
            return self.max_y
        return result

    def x_to_px(self, x):
        percentage = (x - self.min_x) / self.act_width
        result = round(percentage * self.px_width)
        if result < 0:
            return 0
        if result >= self.px_width:
            return self.px_width - 1
        return result

    def y_to_px(self, y):

        percentage = 1 - (y - self.min_y) / self.act_height
        result = round(percentage * self.px_height)
        if result < 0:
            return 0
        if result >= self.px_height:
            return self.px_height - 1
        return result

    def add_raycast(self, position, hit_pos, add_obstacle):
        position_px = self.actual_to_px(position)
        hit_pos_px = self.actual_to_px(hit_pos)
        self.draw_line(position_px, hit_pos_px)
        if add_obstacle:
            self.draw_circle(hit_pos_px)
        self.substract_circle(position_px)

    def draw_line(self, position_px, hit_pos_px):
        assert all(map(lambda x: isinstance(x, int), position_px+hit_pos_px))
        breseham_line = self.bresenham_line(position_px, hit_pos_px)
        for x, y in breseham_line:
            if not (0 <= x < self.px_width and 0 <= y < self.px_height):
                continue

            if self.canvas[y][x] <= 30:
                self.canvas[y][x] = PATH_CLEAR_VALUE
            else:
                self.canvas[y][x] = self.canvas[y][x] - 30


    def draw_circle(self, hit_pos_px):
        addition = np.zeros(self.canvas.shape, dtype=self.canvas.dtype)
        color = 140
        cv2.circle(addition, hit_pos_px, PATH_OBSTACLE_RADIUS, color // 3)
        cv2.circle(addition, hit_pos_px, PATH_OBSTACLE_RADIUS // 3, color)
        self.canvas += addition
        self.canvas[self.canvas < addition] = 255

    def substract_circle(self, center_pos):
        circle_pixels = self.generate_circle_pixels(center_pos, ROBOT_RADIUS)
        for x, y, _ in circle_pixels:
            if x >= self.px_width:
                continue
            if y >= self.px_height:
                continue
            color_decrement = 30
            if self.canvas[y][x] > color_decrement:
                self.canvas[y][x] = self.canvas[y][x] - color_decrement
            else:
                self.canvas[y][x] = PATH_CLEAR_VALUE

    def resize_dilated_but_efficient(self, new_resolution, show_image=False):
        result = NumPyMap(self.map_start, self.map_end, new_resolution, show_image=show_image)
        for x in range(result.px_width):
            for y in range(result.px_height):
                actual_rect_of_ret = result.px_to_actual_rect((x, y))
                px_rect_of_self = self.actual_rect_to_px_rect(actual_rect_of_ret)
                sliced = px_rect_of_self.slice_map(self.canvas)
                result.canvas[y][x] = sliced.max(initial=0)
        return result

    def generate_circle_pixels(self,center, radius):
        square_area_of_radius = (1 + (radius - 1) * 2) ** 2
        square_area_of_radius_min_1 = max(0, (1 + (radius - 2) * 2) ** 2)
        max_number_of_outter_cells = square_area_of_radius - square_area_of_radius_min_1

        number_of_degree_rotation = max_number_of_outter_cells // 2 + 1
        degree_rotation = (math.pi * 2) / number_of_degree_rotation

        prev_x = float('inf')
        for i in range(number_of_degree_rotation):
            degree = degree_rotation * i
            x = center[0] + round(radius * math.cos(degree))
            y1 = center[1] + round(radius * math.sin(degree))
            y0 = center[1] - round(radius * math.sin(degree))
            if x == prev_x:
                continue
            prev_x = x
            for y in range(y0, y1):
                manhattan_dist = abs(x - center[0]) + abs(y - center[1])
                yield x, y, manhattan_dist


    def bresenham_line(self,start, end):
        """Generate coordinates of the line from (x0, y0) to (x1, y1) using Bresenham's algorithm."""
        x0, y0 = start
        x1, y1 = end

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy