import random

class ExploreMap():
    def __init__(self, bfsStrategy, onComplete):
        self.chains = [bfsStrategy]
        self.onComplete = onComplete
        self.numpyMap = None
        self.currentActualPos = None
        self.dirty_bit = True
        self.finished = False

    def get_destination(self):
        if self.finished:
            return True
        chain = self.chains[0]
        curr_destination = chain.get_destination()
        if curr_destination is None:
            curr_destination = self.move_on_to_next_destination()
        if curr_destination is not None:
            return curr_destination
        self.finished = True
        self.onComplete()

    def move_on_to_next_destination(self):
        first_item = self.chains[0]
        result = first_item.move_on_to_next_destination()
        return result

    def overall_destinations(self):
        return self.chains[0].overall_destinations()

    def tick_to_check_if_need_replan(self, robot_pose):
        return self.chains[0].tick_to_check_if_need_replan(robot_pose)

    def reset_dirty_bit(self):
        result = self.dirty_bit
        self.dirty_bit = False
        return result

    def try_set_map(self, numpyMap, currentActualPos, chance):
        assert 0 <= chance <= 100
        if random.randint(0, 100) > chance and self.numpyMap is not None and len(self.overall_destinations()) > 0:
            return True
        result = self.set_map(numpyMap, currentActualPos)
        print(self.overall_destinations())
        return result

    def set_map(self, numpyMap, currentActualPos):
        if self.finished:
            return True
        self.numpyMap = numpyMap
        self.currentActualPos = currentActualPos
        chain = self.chains[0]
        success = chain.set_map(numpyMap, currentActualPos)
        if not success:
            self.finished = True
            self.onComplete()
            return False
        self.dirty_bit = True
        return True

