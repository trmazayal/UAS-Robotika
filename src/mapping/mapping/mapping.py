import math
import time
import rclpy
import itertools
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Empty, Float32MultiArray
from rclpy.qos import QoSProfile, QoSHistoryPolicy

from mapping.constants import *
from mapping.map_visualizer.NumPyMap import NumPyMap
from mapping.exploration.ExploreMap import ExploreMap
from mapping.map_visualizer.PlanningMap import PlanningMap
from mapping.exploration.ExploreBlindMap import ExploreBlindMap
from mapping.ros_serializer.RosSerializer import RosSerializers
from mapping.exploration.ExploreBfs import BFSToReachDestination

class GridMapBuilder(Node):
    def __init__(self):
        super().__init__('grid_map_builder')
        self.subscription_pose = self.create_subscription(Pose2D,
                                                            '/robot_pose',
                                                                self.listener_pose,
                                                                    QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST)
                                                                    )
        self.subscription_cmd_vel = self.create_subscription(Twist,
                                                            '/robot_cmd_vel',
                                                                self.robot_cmd_vel_update,
                                                                    QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST)
                                                                    )
        self.publisher_goal_point = self.create_publisher(Float32MultiArray,
                                                            '/goal_point',
                                                                QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST)
                                                                )
        self.subscription_goal_point_reached = self.create_subscription(Empty,
                                                            '/goal_point/reached',
                                                                self.goal_point_is_reached, 1
                                                                )
        self.subscription_goal_point_blocked = self.create_subscription(Empty,
                                                            '/goal_point/blocked',
                                                                self.goal_point_is_blocked, 1
                                                                )
        self.subscription_goal_point_blocked = self.create_subscription(Empty,
                                                            '/goal_point/redo_bfs',
                                                                self.goal_point_redo_bfs, 1
                                                                )
        self.subscription_scan = self.create_subscription(Float32MultiArray,
                                                            '/goal_point/redo_bfs_if_blocked',
                                                                self.redo_bfs_if_blocked,
                                                                    QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST)
                                                                )
        self.subscription_scan = self.create_subscription(LaserScan,
                                                            '/hokuyo',
                                                                self.listener_scan,
                                                                    QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST)
                                                                )
        self.publisher_map = self.create_publisher(OccupancyGrid, '/occupancy_map', 10)
        self.publisher_map_robot = self.create_publisher(Pose2D, '/occupancy_map/robot_pose', 10)

        self.map = NumPyMap()
        self._resized_map = None
        self.current_pose = None

        fig = plt.figure()
        fig.set_figheight(5)
        fig.set_figwidth(8)
        ax1 = fig.add_subplot()
        fig.canvas.manager.set_window_title('Path Planning Map')

        self.displayer = PlanningMap(self.map, fig, ax1)
        self.displayer.update_frame()

        self.pause_mapping = False
        self.start_time = None

        # Grid map parameters
        self.resolution = 0.01  # meters per cell
        self.map_size_x = self.map_size_y = 10  # meters
        self.grid_size_x, self.grid_size_y = int(self.map_size_x / self.resolution), int(self.map_size_y / self.resolution)
        self.grid_map = np.zeros((self.grid_size_x, self.grid_size_y), dtype=np.int8)

        self.bfs = ExploreMap(
                        ExploreBlindMap(),
                        lambda: self.set_bfs(
                            ExploreMap(
                                BFSToReachDestination(START_NAVIGATION_PLANNING),
                                lambda: self.set_bfs(
                                    ExploreMap(
                                        BFSToReachDestination(FINISH_NAVIGATION_PLANNING),
                                        lambda: print("START MAPPING - STARTING POINT - FINISH POINT - DONE!")
                                    )
                                )
                            )
                        )
                    )

        self.is_processing = False

    def set_bfs(self, new_bfs):
        self.bfs = new_bfs

    def robot_cmd_vel_update(self, msg: Twist) -> None:
        rotation_speed = msg.angular.z
        if self.start_time is None:
            return
        if time.time() < self.start_time + 4:
            return
        self.pause_mapping = abs(rotation_speed) > math.radians(35)


    def goal_point_is_reached(self, *msg):
        print(f"Reached, redo BFS. current: {self.bfs.overall_destinations()}")
        self.broadcast_goal(self.map_for_bfs, 100)

    def goal_point_is_blocked(self, *msg):
        print(f"Blocked, redo BFS. current: {self.bfs.overall_destinations()}")

    def goal_point_redo_bfs(self, *msg):
        print(f"Redoing BFS. current: {self.bfs.overall_destinations()}")
        self.broadcast_goal(self._resized_map, 100)

    def redo_bfs_if_blocked(self, *msg):
        if self.bfs is not None:
            route = self.bfs.overall_destinations()

            if len(route) > 0:
                curr_pos = self.map_for_bfs.actual_to_px((self.current_pose.x, self.current_pose.y))
                next_destination = self.bfs.overall_destinations()[0][2]
                next_destination = self.bresenham_line(curr_pos[0], curr_pos[1], next_destination[0], next_destination[1])
                line_to_next_dest = list(next_destination)
                for route in itertools.chain(map(lambda x: x[2], route), line_to_next_dest):
                    px, py = route
                    if self.map_for_bfs.canvas[py][px] >= PATH_OBSTACLE_TRESHOLD:
                        self.bfs.try_set_map(self.map_for_bfs, (self.current_pose.x, self.current_pose.y), 100)
                        return

    @property
    def map_for_bfs(self):
        return self._resized_map

    def listener_pose(self, msg):
        self.current_pose = msg
        self.start_time = self.start_time if self.start_time is not None else time.time()
        grid_x, grid_y = int((self.current_pose.x / self.resolution) + (self.grid_size_x / 2)), int((self.current_pose.y / self.resolution) + (self.grid_size_y / 2))

        msg = Pose2D()
        msg.x, msg.y = float(grid_x), float(grid_y)
        msg.theta = self.current_pose.theta
        self.publisher_map_robot.publish(msg)

    def listener_scan(self, msg):
        if self.current_pose is None:
            return

        distances = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        x_coords, y_coords = distances * np.cos(angles + self.current_pose.theta) + self.current_pose.x, distances * np.sin(angles + self.current_pose.theta) + self.current_pose.y
        curr_pos = self.current_pose.x, self.current_pose.y

        if not self.pause_mapping:
            for end_x, end_y, distance in zip(x_coords, y_coords, distances):
                self.map.add_raycast(curr_pos, (end_x, end_y), distance < msg.range_max)

        self.map.robot_pos = self.current_pose
        self._resized_map = self.map.resize_dilated_but_efficient(ALGORITHM_RESOLUTION)

        self.displayer.update_frame()

        self.broadcast_goal(self._resized_map)

    def broadcast_goal(self, resized_map, chance=0):
        if self.current_pose is None:
            return
        self.bfs.try_set_map(resized_map, (self.current_pose.x, self.current_pose.y), chance)
        self.displayer.set_destinations(self.bfs.overall_destinations())

        self.bfs.tick_to_check_if_need_replan(self.current_pose)

        if self.bfs.reset_dirty_bit():
            goal_points = self.bfs.overall_destinations()
            self.publisher_goal_point.publish(RosSerializers().serialize(goal_points))
            print(f"Publishing. Current pose: {self._resized_map.actual_to_px((self.current_pose.x, self.current_pose.y))}")

    def bresenham_line(self, x0, y0, x1, y1):
        """Generate coordinates of the line from (x0, y0) to (x1, y1) using Bresenham's algorithm."""
        assert isinstance(x0, int)
        assert isinstance(y0, int)
        assert isinstance(x1, int)
        assert isinstance(y1, int)
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

def main(args=None):
    rclpy.init(args=args)
    grid_map_builder = GridMapBuilder()
    rclpy.spin(grid_map_builder)
    grid_map_builder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
