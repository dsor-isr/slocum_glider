from math import atan2, degrees, sqrt

from pyproj import Geod
from slocum_glider_msgs.msg import (GoToWaypointAction, GoToWaypointFeedback,
                                    GoToWaypointGoal, GoToWaypointResult)

from .base import Behavior
from ..modes import MODE_GOTO_WAYPOINT_BIT
from ..utils import decimal_degs_to_decimal_mins, decimal_mins_to_decimal_degs


C_WPT_TOLERANCE = 1e-6
MAX_CYCLES_TO_WAIT = 5 * 60 * 4  # about 5 minutes


G = Geod(ellps="WGS84")


def waypoint_to_decimal_minutes(g, units, x, y):
    if units == 'decimal_degrees':
        if y:
            lat = decimal_degs_to_decimal_mins(y)
        else:
            lat = g.state.m_lat
        if x:
            lon = decimal_degs_to_decimal_mins(x)
        else:
            lon = g.state.m_lon
    elif units == 'decimal_minutes':
        if y:
            lat = y
        else:
            lat = g.state.m_lat
        if x:
            lon = x
        else:
            lon = g.state.m_lon
    elif units == 'relative':
        if not x:
            x = 0
        if not y:
            y = 0

        azimuth = degrees(atan2(x, y))
        distance = sqrt(x**2 + y**2)
        lon, lat, _ = G.fwd(
            decimal_mins_to_decimal_degs(g.state.m_lon),
            decimal_mins_to_decimal_degs(g.state.m_lat),
            azimuth,
            distance,
            False
        )

        lat = decimal_degs_to_decimal_mins(lat)
        lon = decimal_degs_to_decimal_mins(lon)

    return lon, lat


class GoToWaypointBehavior(Behavior):
    """A behavior that navigates to a waypoint and stays at that
waypoint. Terminates when the waypoint is reached.

Takes three arguments:

+ units: decimal_degrees (default), decimal_minutes, or relative.
+ x: longitude
+ y: latitude
+ dist: acceptance distance (m). defaults to 10m

x and y default to None. If None, they are initialized to the glider's
estimated position when the behavior starts.

    """

    ACTION = GoToWaypointAction
    ACTION_NAME = 'go_to_waypoint'
    CONTROLS = set(['heading'])
    DEFAULT_MODES_ENABLED = [MODE_GOTO_WAYPOINT_BIT]

    def __init__(self, x=None, y=None, units='decimal_degrees', dist=10,
                 server=None):
        super(GoToWaypointBehavior, self).__init__()
        if units != 'decimal_degrees' \
           and units != 'decimal_minutes' \
           and units != 'relative':
            raise ValueError('unknown units: ' + units)
        self.x = x
        self.y = y
        self.dist = dist
        self.units = units
        self.server = server

    @classmethod
    def from_goal(cls, goal, server):
        if goal.units == GoToWaypointGoal.DECIMAL_DEGREES:
            units = 'decimal_degrees'
        elif goal.units == GoToWaypointGoal.DECIMAL_MINUTES:
            units = 'decimal_minutes'
        elif goal.units == GoToWaypointGoal.RELATIVE:
            units = 'relative'
        else:
            raise ValueError('unknown units: ' + str(goal.units))
        return cls(units=units,
                   x=goal.x,
                   y=goal.y,
                   dist=goal.dist,
                   server=server)

    def do_start(self, g):
        # Figure out the coordinates to send to the glider.
        self.lon, self.lat = waypoint_to_decimal_minutes(
            g,
            self.units,
            self.x,
            self.y
        )

        self.num_cycles = 0
        self.num_cycles_waiting = 0

    def do_resume(self, g):
        self.num_cycles = 0
        self.num_cycles_waiting = 0

    def do_step(self, g):
        g.state.u_mission_param_a = self.lon
        g.state.u_mission_param_b = self.lat

        c_wpt_lat = g.state.c_wpt_lat
        c_wpt_lon = g.state.c_wpt_lon

        # Wait until what the glider thinks the waypoint is matches what we
        # commanded.
        if abs(c_wpt_lat - self.lat) >= C_WPT_TOLERANCE \
           or abs(c_wpt_lon - self.lon) >= C_WPT_TOLERANCE:
            self.num_cycles_waiting += 1
            if self.num_cycles_waiting >= MAX_CYCLES_TO_WAIT:
                self.abort(g, text='Waited too long for glider to update wpt')
                return
            else:
                return

        # Now that we and the glider think we're going to the same place, give
        # it some time to update the dist to wpt.
        self.num_cycles += 1
        if self.num_cycles <= 8:
            return
        dist_to_wpt = g.state.m_dist_to_wpt

        if self.server:
            self.server.send_feedback(
                GoToWaypointFeedback(dist_to_goal=dist_to_wpt)
            )
        if dist_to_wpt <= self.dist:
            self.stop(g, text='reached waypoint')

    def do_abort(self, g):
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return GoToWaypointResult()
