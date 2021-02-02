from slocum_glider_msgs.msg import (StayAtWaypointAction, StayAtWaypointGoal,
                                    StayAtWaypointResult)
from utm import from_latlon, to_latlon

from .base import Behavior
from ..modes import MODE_GOTO_WAYPOINT_BIT
from ..utils import decimal_degs_to_decimal_mins, decimal_mins_to_decimal_degs


class StayAtWaypointBehavior(Behavior):
    """A behavior that navigates to a waypoint and stays at that waypoint. Never
terminates unless preempted.

Takes three arguments:

+ units: decimal_degrees (default), decimal_minutes, or relative.
+ x: longitude
+ y: latitude

x and y default to None. If None, they are initialized to the glider's
estimated position when the behavior starts.

    """

    ACTION = StayAtWaypointAction
    ACTION_NAME = 'stay_at_waypoint'
    CONTROLS = set(['heading'])

    def __init__(self, x=None, y=None, units='decimal_degrees', server=None):
        super(StayAtWaypointBehavior, self).__init__()
        if units != 'decimal_degrees' and units != 'decimal_minutes':
            raise ValueError('unknown units: ' + units)
        self.x = x
        self.y = y
        self.units = units
        self.server = server

    @classmethod
    def from_goal(cls, goal, server):
        if goal.units == StayAtWaypointGoal.DECIMAL_DEGREES:
            units = 'decimal_degrees'
        elif goal.units == StayAtWaypointGoal.DECIMAL_MINUTES:
            units = 'decimal_minutes'
        else:
            raise ValueError('unknown units: ' + str(goal.units))
        return cls(units=units,
                   x=goal.x,
                   y=goal.y,
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
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return StayAtWaypointResult()
