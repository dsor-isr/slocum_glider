from .behavior import Behavior


class SensorsInBehavior(Behavior):
    """Control the sensor sampling behavior."""

    NAME = 'sensors_in'

    HAS_WAITING_STATE = False

    def compute_controls(self, x):
        """The b_args correspond directly to sensors, so we can just update the state
and call it done.

        """
        x.update(self.args)

    def should_stop(self, x):
        return True
