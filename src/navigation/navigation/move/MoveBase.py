import time

class MoveBase:
    def __init__(self, end_duration_relative, next_override=None):
        self.end_duration = int(time.time()) + end_duration_relative
        self.next_override = next_override

    @property
    def is_valid(self):
        return int(time.time()) < self.end_duration

    @property
    def twist(self):
        raise NotImplementedError()

    def _get_twist_of_next_override(self):
        if self.next_override is not None:
            return self.next_override.twist
        else:
            return None

    @staticmethod
    def chain(*movement_overrides):
        if len(movement_overrides) == 0:
            return None
        for i in range(len(movement_overrides)):
            if i+1 >= len(movement_overrides):
                break
            movement_overrides[i].next_override = movement_overrides[i+1]
        return movement_overrides[0]