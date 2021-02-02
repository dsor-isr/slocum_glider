from slocum_glider_msgs.msg import (GoToWaypointAction, GoToWaypointFeedback,
                                    GoToWaypointGoal, GoToWaypointResult)
from utm import from_latlon, to_latlon

from .base import Behavior
from ..modes import MODE_GOTO_WAYPOINT_BIT
from ..utils import decimal_degs_to_decimal_mins, decimal_mins_to_decimal_degs


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
        if self.units == 'decimal_degrees':
            if self.y:
                lat = decimal_degs_to_decimal_mins(self.y)
            else:
                lat = g.state.m_lat
            if self.x:
                lon = decimal_degs_to_decimal_mins(self.x)
            else:
                lon = g.state.m_lon
        elif self.units == 'decimal_minutes':
            if self.y:
                lat = self.y
            else:
                lat = g.state.m_lat
            if self.x:
                lon = self.x
            else:
                lon = g.state.m_lon
        elif self.units == 'relative':

            easting, northing, zone_num, zone_char = from_latlon(
                decimal_mins_to_decimal_degs(g.state.m_lat),
                decimal_mins_to_decimal_degs(g.state.m_lon)
            )

            if self.x:
                easting += self.x
            if self.y:
                northing += self.y

            lat, lon = to_latlon(easting, northing, zone_num, zone_char)
            lat = decimal_degs_to_decimal_mins(lat)
            lon = decimal_degs_to_decimal_mins(lon)

        g.change_modes([MODE_GOTO_WAYPOINT_BIT], [])
        g.state.u_mission_param_a = lon
        g.state.u_mission_param_b = lat
        self.num_cycles = 0

    def do_step(self, g):
        self.num_cycles += 1
        if self.num_cycles <= 4:
            # Give the glider some time to compute the distance to the
            # waypoint.
            return
        dist_to_wpt = g.state.m_dist_to_wpt

        if self.server:
            self.server.send_feedback(
                GoToWaypointFeedback(dist_to_goal=dist_to_wpt)
            )
        if dist_to_wpt <= self.dist:
            self.stop(g, text='reached waypoint')

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return GoToWaypointResult()
