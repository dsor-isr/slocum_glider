from .behavior import Behavior


class AbendBehavior(Behavior):
    NAME = 'abend'
    HAS_WAITING_STATE = False

    def compute_controls(self, x):
        pass
