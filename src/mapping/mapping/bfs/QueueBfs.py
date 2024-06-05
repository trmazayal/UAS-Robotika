import collections

from mapping.constants import *

class QueueBfs():
    def __init__(self):
        self.queue = collections.deque()
        self.currentDistance = 0
        self.new_distance(initializeOnly=True)
        self.currentItem = self.queue.popleft()

    def push(self, index, item):
        self.lastItem[index].append(item)

    def new_distance(self, initializeOnly=False):
        self.lastItem = []
        for i in range(TOTAL_DIRECTION):
            self.lastItem.append([])

        self.queue.append(self.lastItem)
        if not initializeOnly:
            self.currentDistance += 1

    def is_current_distance_empty(self):
        return all(map(lambda x: len(x) == 0, self.currentItem))

    def pop_item_at_current_distance(self, index):
        for i in range(TOTAL_DIRECTION // 2 + 1):
            curr_direction = (index + i) % TOTAL_DIRECTION
            curr_item = self.currentItem[curr_direction]
            if len(curr_item) > 0:
                return curr_item.pop(), curr_direction
            curr_direction = (index - i) % TOTAL_DIRECTION
            curr_item = self.currentItem[curr_direction]
            if len(curr_item) > 0:
                return curr_item.pop(), curr_direction
        return None

    def to_next_distance(self):
        self.currentItem = self.queue.popleft()



