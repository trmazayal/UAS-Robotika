from navigation.move.MoveBase import MoveBase

class MoveNoOperation(MoveBase):
    def __init__(self, _ignored=None, next_override=None):
        super().__init__(-1)

    @property
    def twist(self):
        return
