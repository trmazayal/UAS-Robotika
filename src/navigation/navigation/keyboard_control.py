import rclpy

from pynput.keyboard import Key, Listener

from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboardcontrol')
        self.subscriber_robot_pose = self.create_subscription(Pose2D,
                                                                '/robot_pose',
                                                                    self.robot_pose_callback, qos_profile=10)
        self.publisher_goal_point = self.create_publisher(Point, '/goal_point', 10)

        self.timer = self.create_timer(1, self.keyboardcontrols)

        self.robot_pose = Pose2D()
        self.robot_pose_received = False
        self.direction_length = 4
        self.is_moving = False

    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_received = True

    def new_goal_point(self, x, y, z=0.0):
        new_goal_point = Point()
        new_goal_point.x, new_goal_point.y, new_goal_point.z = x, y, z
        return new_goal_point

    def move(self, direction: str, x, y, z=0.0):
        dx = self.robot_pose.x + x
        dy = self.robot_pose.y + y
        new_goal_point = self.new_goal_point(dx, dy)
        self.publisher_goal_point.publish(new_goal_point)
        print(f"Move {direction}")
        self.is_moving = True

    def stop(self):
        new_goal_point = self.new_goal_point(self.robot_pose.x, self.robot_pose.y)
        self.publisher_goal_point.publish(new_goal_point)
        print("Stop")
        self.is_moving = False

    def keyboardcontrols(self):
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()
        return True

    def on_press(self,key):
        if key.char == 'w':
            self.move("forward", 0, self.direction_length)
        elif key.char == 's':
            self.move("backward", 0, -self.direction_length)
        elif key.char == 'a':
            self.move("left", -self.direction_length, 0)
        elif key.char == 'd':
            self.move("right", self.direction_length, 0)
        return True

    def on_release(self,key):
        self.stop()
        return True

def main(args=None):
    rclpy.init(args=args)
    keyboardcontrol = KeyboardControl()
    rclpy.spin(keyboardcontrol)
    keyboardcontrol.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()