from geometry_msgs.msg import Twist

import math

from navigation.move.MoveBase import MoveBase
from navigation.constants import *

class MoveBackward(MoveBase):
    def __init__(self, obstacle_on_left, expire_duration=3, speed_multiplier=1, next_override=None):
        super().__init__(expire_duration)
        self.next_override = next_override
        self.obstacle_at_left = obstacle_on_left
        self.speed_multiplier = speed_multiplier

    @property
    def twist(self):
        if not self.is_valid:
            return self._get_twist_of_next_override()
        result = Twist()
        theta = math.radians(-DEGREE if self.obstacle_at_left else DEGREE) * 3
        result.linear.y = -MOVEMENT_SPEED * self.speed_multiplier
        result.angular.z = 2.0 * theta
        return result