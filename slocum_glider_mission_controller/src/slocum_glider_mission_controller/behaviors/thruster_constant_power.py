from slocum_glider_msgs.msg import (ThrusterConstantPowerAction,
                                    ThrusterConstantPowerResult)

from .base import Behavior


class ThrusterConstantPowerBehavior(Behavior):
    """Set the thruster to operate at a constant power draw. Never terminates
unless preempted.

Takes one argument:

+ power :: Power in Watts.
    """

    ACTION = ThrusterConstantPowerAction
    ACTION_NAME = 'thruster_constant_power'
    CONTROLS = set(['thruster'])

    def __init__(self, power=5, server=None):
        super(ThrusterConstantPowerBehavior, self).__init__()
        self.power = power
        self.server = server

    @classmethod
    def from_goal(cls, goal, server):
        return cls(power=goal.power,
                   server=server)

    def do_start(self, g):
        g.state.u_mission_param_g = self.power
        g.state.u_mission_param_i = self.power

    def do_step(self, g):
        pass

    def do_abort(self, g):
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return ThrusterConstantPowerResult()
