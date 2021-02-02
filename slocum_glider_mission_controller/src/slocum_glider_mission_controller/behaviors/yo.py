from slocum_glider_msgs.msg import YoAction, YoResult

from .base import Behavior
from ..modes import MODE_TURN_TO_PORT_BIT, MODE_TURN_TO_STARBOARD_BIT


class YoBehavior(Behavior):
    """Yos the glider within a constant depth band and at a constant pitch. Never
terminates unless preempted.

Takes 5 parameters.

+ dive_depth (defaults to 20m)
+ dive_altitude (defaults to 5m)
+ dive_pitch (defaults to -0.4536 rad/-26 deg)
+ climb_depth (defaults to 2m)
+ climb_pitch (defaults to 0.4536 rad/26 deg)

    """

    ACTION = YoAction
    ACTION_NAME = 'yo'
    CONTROLS = set(['pitch', 'bpump'])

    def __init__(self, dive_depth=20, dive_altitude=5, dive_pitch=-0.4536,
                 climb_depth=2, climb_pitch=0.4536, server=None):
        super(YoBehavior, self).__init__()
        self.dive_depth = dive_depth
        self.dive_altitude = dive_altitude
        self.dive_pitch = dive_pitch
        self.climb_depth = climb_depth
        self.climb_pitch = climb_pitch
        self.server = server

    @classmethod
    def from_goal(cls, goal, server):
        return cls(dive_depth=goal.dive_depth,
                   dive_altitude=goal.dive_altitude,
                   dive_pitch=goal.dive_pitch,
                   climb_depth=goal.climb_depth,
                   climp_pitch=goal.climb_pitch,
                   server=server)

    def do_start(self, g):
        g.change_modes([], [MODE_TURN_TO_PORT_BIT, MODE_TURN_TO_STARBOARD_BIT])
        g.state.u_mission_param_c = self.dive_depth
        g.state.u_mission_param_d = self.dive_altitude
        g.state.u_mission_param_e = self.climb_depth
        g.state.u_mission_param_f = self.dive_pitch
        g.state.u_mission_param_h = self.climb_pitch

    def do_step(self, g):
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return YoResult()
