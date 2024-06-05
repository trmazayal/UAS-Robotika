from geometry_msgs.msg import Twist

from navigation.move.MoveBase import MoveBase

class Rotating(MoveBase):
    def __init__(self, angular_velocity_func, end_duration_relative=3, next_override=None):
        super().__init__(end_duration_relative, next_override)
        self.angular_velocity_func = angular_velocity_func

    @property
    def twist(self):
        if not self.is_valid:
            return self._get_twist_of_next_override()
        cmd_vel = Twist()
        cmd_vel.linear.y = 0.0

        angular_velocity = self.angular_velocity_func()
        if angular_velocity is None:
            self.end_duration = 0
            return self._get_twist_of_next_override()
        cmd_vel.angular.z = float(angular_velocity)
        return cmd_vel