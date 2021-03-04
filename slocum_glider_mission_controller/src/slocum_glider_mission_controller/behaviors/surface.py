from slocum_glider_msgs.msg import SurfaceAction, SurfaceResult

from .base import Behavior
from ..modes import MODE_NORMAL_SURFACE_BIT


class SurfaceBehavior(Behavior):
    """A behavior that brings the glider to the surface.

Uses the glider's yo behavior to drive near the surface before triggering the
glider's surface behavior.

Takes 2 parameters.

+ climb_depth (defaults to 2m)
+ climb_pitch (defaults to 0.4536 rad/26 deg)
    """

    ACTION = SurfaceAction
    ACTION_NAME = 'surface'
    CONTROLS = set(['pitch', 'bpump'])

    def __init__(self, climb_depth=2, climb_pitch=0.4536, server=None):
        super(SurfaceBehavior, self).__init__()
        self.server = server
        self.climb_depth = climb_depth
        self.climb_pitch = climb_pitch

    @classmethod
    def from_goal(cls, goal, server):
        return cls(climb_depth=goal.climb_depth,
                   climb_pitch=goal.climb_pitch,
                   server=server)

    def do_start(self, g):
        self.substate = 'START'

    def do_step(self, g):

        if self.substate == 'START':
            # Trigger an inflection immediately
            g.state.u_mission_param_c = g.state.m_depth - 1
            g.state.u_mission_param_e = max(self.climb_depth - 5, 1)
            g.state.u_mission_param_h = self.climb_pitch
            self.substate = 'CLIMBING'
        elif self.substate == 'CLIMBING':
            g.state.u_mission_param_e = max(self.climb_depth - 5, 1)
            g.state.u_mission_param_h = self.climb_pitch
            if (g.state.m_depth <= self.climb_depth):
                self.substate = 'SURFACING'
        elif self.substate == 'SURFACING':
            self.enable_mode(MODE_NORMAL_SURFACE_BIT)
            self.substate = 'WAITING'
        elif self.substate == 'WAITING':
            if g.state.x_in_surface_dialog:
                # We need to exit the surface mode otherwise we'll never stop
                # trying to surface!
                self.disable_mode(MODE_NORMAL_SURFACE_BIT)
                self.substate = 'IN_DIALOG'
        elif self.substate == 'IN_DIALOG':
            if not g.state.x_in_surface_dialog:
                self.stop(g)
                self.substate = 'DONE'

    def do_abort(self, g):
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return SurfaceResult()
