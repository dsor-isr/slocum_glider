from slocum_glider_msgs.msg import ThrusterOffAction, ThrusterOffResult

from .base import Behavior


class ThrusterOffBehavior(Behavior):
    """Shuts off the thruster. Never terminates unless preempted."""

    ACTION = ThrusterOffAction
    ACTION_NAME = 'thruster_off'
    CONTROLS = set(['thruster'])

    def __init__(self, server=None):
        super(ThrusterOffBehavior, self).__init__()
        self.server = server

    @classmethod
    def from_goal(cls, goal, server):
        return cls(server=server)

    def do_start(self, g):
        pass

    def do_step(self, g):
        g.state.u_mission_param_g = 1
        g.state.u_mission_param_i = 1

    def do_abort(self, g):
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return ThrusterOffResult()
