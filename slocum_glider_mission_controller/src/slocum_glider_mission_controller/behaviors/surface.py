import rospy
from slocum_glider_msgs.msg import SurfaceAction, SurfaceResult

from .base import Behavior
from ..modes import MODE_NORMAL_SURFACE_BIT, MODE_NORMAL_SURFACE_RESUME_BIT

MIN_DEPTH = 3


class SurfaceBehavior(Behavior):
    """A behavior that brings the glider to the surface.

Uses the glider's yo behavior to drive near the surface before triggering the
glider's surface behavior.

Takes 2 parameters.

+ climb_depth (defaults to 3m)
+ climb_pitch (defaults to 0.4536 rad/26 deg)
+ end_action (defaults to "ctrl-c-resume") either "ctrl-c-resume" to print
  surface dialog and wait for operator interaction before continuing or
  "resume" to resume the mission without waiting for operator input.
    """

    ACTION = SurfaceAction
    ACTION_NAME = 'surface'
    CONTROLS = set(['pitch', 'bpump'])

    def __init__(self, climb_depth=3, climb_pitch=0.4536,
                 end_action="ctrl-c-resume", server=None):
        super(SurfaceBehavior, self).__init__()
        self.server = server
        self.climb_depth = climb_depth
        self.climb_pitch = climb_pitch
        self.end_action = end_action

    @classmethod
    def from_goal(cls, goal, server):
        return cls(climb_depth=goal.climb_depth,
                   climb_pitch=goal.climb_pitch,
                   server=server)

    def do_start(self, g):
        self.substate = 'START'

    def do_step(self, g):

        rospy.logdebug('SURFACING: Substate: %s', self.substate)
        if self.substate == 'START':
            # Trigger an inflection immediately
            d_target_depth = max(g.state.m_depth - 1, MIN_DEPTH + 1)
            c_target_depth = max(self.climb_depth - 5, MIN_DEPTH)

            g.state.u_mission_param_c = d_target_depth
            g.state.u_mission_param_e = c_target_depth

            rospy.logdebug('SURFACING: c_target_depth: %s d_target_depth: %s',
                           c_target_depth, d_target_depth)
            g.state.u_mission_param_h = self.climb_pitch
            self.substate = 'CLIMBING'
        elif self.substate == 'CLIMBING':
            g.state.u_mission_param_e = max(self.climb_depth - 5, MIN_DEPTH)
            g.state.u_mission_param_h = self.climb_pitch
            if (g.state.m_depth <= self.climb_depth):
                self.substate = 'SURFACING'
        elif self.substate == 'SURFACING':
            if self.end_action == 'resume':
                self.enable_mode(MODE_NORMAL_SURFACE_RESUME_BIT)
            else:
                self.enable_mode(MODE_NORMAL_SURFACE_BIT)
            self.substate = 'WAITING_FOR_BEHAVIOR_TO_ACTIVATE'
        elif self.substate == 'WAITING_FOR_BEHAVIOR_TO_ACTIVATE':
            if g.state.m_surface_depth_reached and g.state.m_surfacing:
                # We need to exit the surface mode otherwise we'll never stop
                # trying to surface!
                self.disable_mode(MODE_NORMAL_SURFACE_BIT)
                self.disable_mode(MODE_NORMAL_SURFACE_RESUME_BIT)
                self.substate = 'WAITING_FOR_BEHAVIOR_TO_END'
        elif self.substate == 'WAITING_FOR_BEHAVIOR_TO_END':
            if not g.state.m_surface_depth_reached or not g.state.m_surfacing:
                self.stop(g)
                self.substate = 'DONE'

    def do_abort(self, g):
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return SurfaceResult()
