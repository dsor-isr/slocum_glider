from slocum_glider_msgs.msg import (StayAtWaypointAction, StayAtWaypointGoal,
                                    StayAtWaypointResult)

from .base import Behavior
from .go_to_waypoint import waypoint_to_decimal_minutes
from ..modes import MODE_GOTO_WAYPOINT_BIT


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
    MODES_ENABLED = [MODE_GOTO_WAYPOINT_BIT]

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
        elif goal.units == StayAtWaypointGoal.RELATIVE:
            units = 'relative'
        else:
            raise ValueError('unknown units: ' + str(goal.units))
        return cls(units=units,
                   x=goal.x,
                   y=goal.y,
                   server=server)

    def do_start(self, g):
        # Figure out the coordinates to send to the glider.
        lon, lat = waypoint_to_decimal_minutes(
            g,
            self.units,
            self.x,
            self.y
        )

        g.state.u_mission_param_a = lon
        g.state.u_mission_param_b = lat
        self.num_cycles = 0

    def do_step(self, g):
        pass

    def do_abort(self, g):
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return StayAtWaypointResult()
