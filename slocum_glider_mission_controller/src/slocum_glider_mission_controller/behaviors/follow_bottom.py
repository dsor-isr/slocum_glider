from slocum_glider_msgs.msg import FollowBottomAction, FollowBottomResult

from .base import Behavior


class FollowBottomBehavior(Behavior):
    """Yos the glider, attempting to stay within a specified altitude range of the
bottom, while also respecting a depth range and minimum depth band
size. Abnormally terminates if the constraints cannot be met.

+ min_depth (defaults to 2m)
+ max_depth (defaults to 100m)
+ min_altitude (defaults to 5m)
+ max_altitude (defaults to 20m)
+ min_depth_band (defaults to 5m)
+ use_altimeter (defaults to True)
+ climb_pitch (defaults to 0.4536 rad/26 deg)
+ dive_pitch (defaults to -0.4536 rad/-26 deg)

    """

    ACTION = FollowBottomAction
    ACTION_NAME = 'follow_bottom'
    CONTROLS = set(['pitch', 'bpump'])

    def __init__(self, min_depth=2, max_depth=100, min_altitude=5,
                 max_altitude=20, min_depth_band=5, use_altimeter=True,
                 climb_pitch=0.4536, dive_pitch=-0.4536,
                 server=None):
        super(FollowBottomBehavior, self).__init__()
        if not use_altimeter:
            raise ValueError('use_altimeter must be True')
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.min_depth_band = min_depth_band
        self.use_altimeter = use_altimeter
        self.dive_pitch = dive_pitch
        self.climb_pitch = climb_pitch
        self.server = server

    @classmethod
    def from_goal(cls, goal, server):
        return cls(min_depth=goal.min_depth,
                   max_depth=goal.max_depth,
                   min_altitude=goal.min_altitude,
                   max_altitude=goal.max_altitude,
                   min_depth_band=goal.min_depth_band,
                   dive_pitch=goal.dive_pitch,
                   climp_pitch=goal.climb_pitch,
                   use_altimeter=goal.use_altimeter,
                   server=server)

    def do_start(self, g):
        self.last_climb_depth = self.max_depth - self.min_depth_band

    def do_step(self, g):
        g.state.u_mission_param_d = self.min_altitude
        g.state.u_mission_param_f = self.dive_pitch
        g.state.u_mission_param_h = self.climb_pitch

        alt = g.state.m_altitude
        depth = g.state.m_depth

        if g.state.m_altimeter_status == 0:
            dive_depth = depth + alt - self.min_altitude
            climb_depth = depth + alt - self.max_altitude
        else:
            dive_depth = self.max_depth
            climb_depth = self.last_climb_depth

        dive_depth = min(dive_depth, self.max_depth)
        climb_depth = max(min(climb_depth, dive_depth - self.min_depth_band),
                          self.min_depth)

        depth_band = dive_depth - climb_depth

        self.last_climb_depth = climb_depth

        if depth_band < self.min_depth_band:
            self.abort('depth band too small')
        else:
            g.state.u_mission_param_c = dive_depth
            g.state.u_mission_param_e = climb_depth

    def do_abort(self, g):
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return FollowBottomResult()
