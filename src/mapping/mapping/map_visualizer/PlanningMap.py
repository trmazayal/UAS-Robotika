import cv2
import numpy as np
import matplotlib.pyplot as plt

from mapping.constants import *
from mapping.map_visualizer.NumPyMap import NumPyMap


class PlanningMap:
    def __init__(self, map: NumPyMap, fig=None, ax=None):
        self.map = map
        self.destinations = []

        if fig is None:
            self.fig = plt.figure()
        else:
            self.fig = fig

        if ax is None:
            self.ax = self.fig.add_subplot(2, 1, 1)
        else:
            self.ax = ax

        self._canvas_axes = self.ax.imshow(self.map.canvas, vmin=0, vmax=255)
        self.fig.show()

    def update_frame(self):
        copied_canvas = self.img_canvas_processing(self.apply_thresholding(self.map.canvas))
        self.draw_destinations(copied_canvas)

        if self.map.robot_pos is not None:
            copied_canvas = self.display_robot_on_canvas(copied_canvas, self.map)

        self._canvas_axes.set_data(copied_canvas)
        plt.pause(0.001)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def set_destinations(self, destinations):
        self.destinations = destinations

    def draw_destinations(self, copied_canvas):
        for i in range(len(self.destinations)-1):
            (prev_act_x, prev_act_y, _) = self.destinations[i]
            (act_x, act_y, _) = self.destinations[i+1]
            prev_px_x, prev_px_y = self.map.actual_to_px((prev_act_x, prev_act_y))
            px_x, px_y = self.map.actual_to_px((act_x, act_y))
            cv2.line(copied_canvas, (prev_px_x,prev_px_y),(px_x, px_y), WAYPOINT_LINE_COLOR, THICKNESS_WAYPOINT_LINE)
            cv2.circle(copied_canvas, (px_x, px_y), WAYPOINT_RADIUS, WAYPOINT_DOTTED_COLOR, THICKNESS_WAYPOINT_DOTTED)

    def img_canvas_processing(self,image):
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    def apply_thresholding(self,np_canvas_array, copy=True):
        canvas = np.copy(np_canvas_array) if copy else np_canvas_array
        upper_threshold = PATH_OBSTACLE_TRESHOLD
        canvas[np_canvas_array >= upper_threshold] = 255
        canvas[(1 <= np_canvas_array) & (np_canvas_array < upper_threshold)] = 1
        canvas[np_canvas_array == 0] = 124
        return canvas

    def display_robot_on_canvas(self,np_canvas_array, map, copy=True):
        canvas = np.copy(np_canvas_array) if copy else np_canvas_array
        raw_x, raw_y = map.robot_pos.x, map.robot_pos.y
        px, py = map.x_to_px(raw_x), map.y_to_px(raw_y)

        robot_pixel_radius = int(ROBOT_RADIUS * (ROBOT_DEFAULT_RES/map.resolution))
        cv2.circle(canvas, (px, py), robot_pixel_radius, ROBOT_DOTTED_COLOR, -1)

        return canvas