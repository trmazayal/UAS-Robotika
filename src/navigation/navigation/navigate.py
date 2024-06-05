import math
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float32MultiArray
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from geometry_msgs.msg import Pose2D, Point, Twist

from navigation.constants import *
from navigation.move.MoveBase import MoveBase
from navigation.move.MoveRotating import Rotating
from navigation.move.MoveBackward import MoveBackward
from navigation.move.MoveNoOperation import MoveNoOperation
from navigation.sensor.RaycastSensor import RaycastSensor
from navigation.sensor.ReceiveLidarSensor import ReceiveLidarSensor


class Navigate(Node):
    def __init__(self):
        super().__init__('navigate')
        self.subscriber_robot_pose = self.create_subscription(Pose2D,
                                                                '/robot_pose',
                                                                    self.robot_pose_callback,
                                                                        QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST)
                                                                        )
        self.publisher_cmd_vel = self.create_publisher(Twist, '/robot_cmd_vel', 10)
        self.subscriber_goal_point = self.create_subscription(Float32MultiArray,
                                                                '/goal_point',
                                                                    self.goal_point_callback,
                                                                        QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST)
                                                                        )
        self.subscription_scan = self.create_subscription(LaserScan, '/hokuyo', self.listener_scan, 10)
        self.publisher_goal_point_reached = self.create_publisher(Empty, '/goal_point/reached', 10)
        self.publisher_goal_point_blocked = self.create_publisher(Empty, '/goal_point/blocked', 10)
        self.publisher_goal_point_redo_bfs = self.create_publisher(Empty, '/goal_point/redo_bfs', 2)

        self.timer = self.create_timer(TICK_RATE, self.navigate)
        self.vision_blocked_checker = ReceiveLidarSensor(math.radians(10))
        self.straight_raycast = RaycastSensor(OBSTACLE_AVOID_RAYCAST_LENGTH)
        self.movement_override = MoveNoOperation()

        self.laser_scan = LaserScan()
        self.robot_pose = Pose2D()
        self.goal_points = Float32MultiArray()

        self.arrived = True
        self.laser_scan_received = False
        self.goal_point_received = False
        self.robot_pose_received = False
        self.goal_points = []

        self.backward_penalty = 0
        self.goal_point_index = 0
        self.redo_bfs()

    def listener_scan(self, msg):
        self.laser_scan = msg
        self.laser_scan_received = True

    def user_input(self, msg):
        self.movement_override = Rotating(3, 2)

    def redo_bfs(self):
        self.publisher_goal_point_redo_bfs.publish(Empty())

    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_received = True

        if not self.goal_point_received:
            self.goal_points = [self.robot_pose.x, self.robot_pose.y]

    def goal_point_callback(self, msg):
        self.arrived = False
        self.goal_point_index = 0
        self.goal_points = msg.data
        self.goal_point_received = True
        self.__update_current_goal_point_index()
        print(f"Received goal points - Currently Index:{self.goal_point_index}")

    def get_goal_point(self, index):
        return self.goal_points[index*2], self.goal_points[index*2+1]

    def next_goal_point(self):
        if (self.goal_point_index+1)*2 >= len(self.goal_points):
            self.handle_arrived()
            print("No more goal points - Arrived")
            return
        self.goal_point_index += 1
        print(f"Next goal point: {self.goal_point_index} {self.get_goal_point(self.goal_point_index)}")

    def update_straight_raycast(self):
        if not self.laser_scan_received:  # Obstacle Avoidance
            return False

        self.straight_raycast.raycast_length = max(OBSTACLE_AVOID_RAYCAST_LENGTH, 0.5 + self.distance(self.goal_point, self.position))
        distances = np.array(self.laser_scan.ranges)
        angles = np.linspace(self.laser_scan.angle_min, self.laser_scan.angle_max, len(self.laser_scan.ranges))

        self.straight_raycast.obstacle_hits = distances, angles
        self.vision_blocked_checker.set_obstacle_distances(distances)
        self.vision_blocked_checker.set_obstacle_angles(angles)

    def goal_angle(self, relative_to_pov=True, angle_range_start=-math.pi, angle_range_end=math.pi + 0.001):
        dx, dy = self.goal_point.x - self.robot_pose.x, self.goal_point.y - self.robot_pose.y
        goal_angle = np.arctan2(dy, dx)
        if relative_to_pov:
            goal_angle -= self.robot_pose.theta
        while goal_angle < angle_range_start:
            goal_angle += math.pi*2
        while goal_angle > angle_range_end:
            goal_angle -= math.pi*2
        return goal_angle

    def navigate(self):
        twist = self.movement_override.twist
        if twist is not None:
            self.publisher_cmd_vel.publish(twist)
            return
        if  self.goal_distance() < DISTANCE_THRESHOLD_TO_GOAL and math.degrees(abs(self.goal_angle())) < 8:
            self.publisher_cmd_vel.publish(Twist())
            self.next_goal_point()
            return

        goal_angle_deg = math.degrees(self.goal_angle())
        if abs(goal_angle_deg) > MAX_ANGLE_DEGREE_TOWARD_GOAL:
            self.movement_override = self.__get_rotate_in_place()

        dx, dy = self.goal_point.x - self.robot_pose.x, self.goal_point.y - self.robot_pose.y
        distance = self.goal_distance()
        self.update_straight_raycast()

        goal_angle = np.arctan2(dy, dx)

        theta = goal_angle - self.robot_pose.theta
        goal_angle_adjusted = goal_angle
        if goal_angle_adjusted < 0:
            goal_angle_adjusted = goal_angle % math.pi

        while theta > np.pi:
            theta -= 2 * np.pi
        while theta < -np.pi:
            theta += 2 * np.pi

        closest_overall = self.straight_raycast.closest_overall
        self.backward_penalty = max(0, self.backward_penalty-1)
        if closest_overall is not None:
            obstacle_at_left = closest_overall[0] < 0
            magnitude = 9 - 4 * closest_overall[1]/OBSTACLE_AVOID_RAYCAST_LENGTH
            magnitude *= max(1,SPEED_MULTIPLIER)
            magnitude = max(1, magnitude)

            if obstacle_at_left and theta > 0:  # if obstacle on left and we're heading left
                theta = math.radians(-5 * magnitude - 5)  # go to slightly right
            elif not obstacle_at_left and theta < 0:  # if obstacle on right and we're heading right
                theta = math.radians(5 * magnitude + 5)  # go to slightly left

            max_backward_duration = 3/TICK_RATE
            stop_backward_duration = 3/TICK_RATE
            condition = closest_overall[1] < GO_BACKWARD_OBSTACLE_DIST and abs(closest_overall[0]) <= GO_BACKWARD_ROBOT_RADIUS

            if condition and self.backward_penalty < max_backward_duration:
                self.backward_penalty += 2
                self.robot_go_bakcward(obstacle_at_left)
                self.redo_bfs()
            elif condition and self.backward_penalty > max_backward_duration:
                self.backward_penalty += min(max_backward_duration+stop_backward_duration, stop_backward_duration)

        cmd_vel = Twist()

        if distance > DISTANCE_THRESHOLD_TO_GOAL:
            cmd_vel.linear.y = 0.6  * SPEED_MULTIPLIER
            cmd_vel.angular.z = 2.0 * theta
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.next_goal_point()

        self.publisher_cmd_vel.publish(cmd_vel)

        return True

    def goal_distance(self):
        dx, dy = self.goal_point.x - self.robot_pose.x, self.goal_point.y - self.robot_pose.y
        return np.sqrt(dx ** 2 + dy ** 2)

    def handle_arrived(self):
        if self.goal_distance() > DISTANCE_THRESHOLD_TO_GOAL:
            return
        if self.arrived:  # if already arrived since previous iteration
            return
        print("Arrived")
        self.publisher_goal_point_reached.publish(Empty())
        self.arrived = True

    def distance(self,point1, point2):
        if point1 is None:
            return -1
        if point2 is None:
            return -1
        if isinstance(point1, Point):
            point1 = (point1.x, point1.y)
        if isinstance(point2, Point):
            point2 = (point2.x, point2.y)

        dx, dy = point1[0] - point2[0], point1[1] - point2[1]
        result = math.sqrt(dx ** 2 + dy ** 2)
        return result

    def robot_go_bakcward(self, obstacle_left):
        obstacle_position = 'left' if obstacle_left else 'right'
        self.get_logger().info(f"Move Backward - obstacle at the {obstacle_position}")

        expire_duration = 0
        self.movement_override = MoveBase.chain(
            MoveBackward(obstacle_left, expire_duration:= expire_duration + 1, speed_multiplier=0.6), self.__get_rotate_in_place(-10),
        )
        return True

    def __get_rotate_in_place(self, max_angle=MAX_ANGLE_DEGREE_TOWARD_GOAL):
        goal_angle_deg = math.degrees(self.goal_angle())
        def rotation():
            deg = math.degrees(self.goal_angle())
            if abs(deg) <= max_angle:
                return None
            return math.copysign(deg/24, deg)
        print(f"Rotating -> {goal_angle_deg}")
        return Rotating(rotation)

    def __update_current_goal_point_index(self):
        number_of_path_planning = self.goal_points_length
        if number_of_path_planning < 2:
            return
        all_target = np.empty((number_of_path_planning, 2))
        for i in range(number_of_path_planning):
            x, y = self.get_goal_point(i)
            all_target[i, 0] = x
            all_target[i, 1] = y
        for i in range(all_target.shape[0]-1, -1, -1):
            if not self.vision_blocked_checker.is_vision_blocked(all_target[i, :], self.robot_pose):
                self.goal_point_index = i
                return

    @property
    def goal_points_length(self):
        result = len(self.goal_points)
        assert result % 2 == 0
        return result // 2

    @property
    def position(self):
        if not self.robot_pose_received:
            return None
        return self.robot_pose.x, self.robot_pose.y

    @property
    def goal_point(self):
        result = Point()
        if len(self.goal_points):
            result.x, result.y = self.get_goal_point(self.goal_point_index)
        else:
            result.x, result.y = self.robot_pose.x, self.robot_pose.y
        return result

def main(args=None):
    rclpy.init(args=args)
    navigate = Navigate()
    rclpy.spin(navigate)
    navigate.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
