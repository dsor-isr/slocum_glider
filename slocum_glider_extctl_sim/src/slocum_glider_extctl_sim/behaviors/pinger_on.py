from .behavior import Behavior


class PingerOnBehavior(Behavior):
    """Behavior to control the pinger."""
    NAME = 'pinger_on'
    HAS_WAITING_STATE = False

    def compute_controls(self, x):
        """The b_args correspond directly to sensors, so we can just update the state
and call it done.

        """
        x.update(self.args)

    def should_stop(self, x):
        return True
