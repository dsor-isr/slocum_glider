from slocum_glider_msgs.msg import (GoToWaypointListAction,
                                    GoToWaypointListFeedback,
                                    GoToWaypointListResult, Waypoint)

from .base import Behavior
from .go_to_waypoint import waypoint_to_decimal_minutes
from ..modes import MODE_GOTO_WAYPOINT_BIT


class GoToWaypointListBehavior(Behavior):
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

    ACTION = GoToWaypointListAction
    ACTION_NAME = 'go_to_waypoint_list'
    CONTROLS = set(['heading'])
    DEFAULT_MODES_ENABLED = [MODE_GOTO_WAYPOINT_BIT]

    def __init__(self, default_units='decimal_degrees', default_dist=10,
                 waypoints=[], server=None):
        super(GoToWaypointListBehavior, self).__init__()
        self.default_units = default_units
        self.default_dist = default_dist
        self.waypoints = list(waypoints)
        self.current_waypoint = 0
        self.previous_waypoint = -1
        self.current_lon = None
        self.current_lat = None
        self.server = server

    @classmethod
    def from_goal(cls, goal, server):
        waypoints = []
        for wpt in goal.waypoints:
            if wpt.units == Waypoint.DECIMAL_DEGREES:
                units = 'decimal_degrees'
            elif wpt.units == Waypoint.DECIMAL_MINUTES:
                units = 'decimal_minutes'
            elif wpt.units == Waypoint.RELATIVE:
                units = 'relative'
            else:
                raise ValueError('unknown units: ' + str(wpt.units))

            waypoints.append({'x': wpt.x,
                              'y': wpt.y,
                              'units': units,
                              'dist': wpt.dist})

        return cls(waypoints=waypoints, server=server)

    def do_start(self, g):
        self.num_cycles = 0

    def do_resume(self, g):
        self.num_cycles = 0
        # We need to resend the waypoint.
        g.state.u_mission_param_a = self.current_lon
        g.state.u_mission_param_b = self.current_lat

    def do_step(self, g):
        wpt = self.waypoints[self.current_waypoint]
        dist = self.waypoints[self.current_waypoint].get('dist',
                                                         self.default_dist)

        if self.previous_waypoint != self.current_waypoint:
            lon, lat = waypoint_to_decimal_minutes(
                g,
                wpt.get('units', self.default_units),
                wpt['x'],
                wpt['y']
            )
            self.current_lon = lon
            self.current_lat = lat
            g.state.u_mission_param_a = lon
            g.state.u_mission_param_b = lat
            self.previous_waypoint = self.current_waypoint

        self.num_cycles += 1
        if self.num_cycles <= 8:
            # Give the glider some time to compute the distance to the
            # waypoint.
            return
        dist_to_wpt = g.state.m_dist_to_wpt

        if self.server:
            self.server.send_feedback(
                GoToWaypointListFeedback(
                    current_waypoint=self.current_waypoint,
                    dist_to_goal=dist_to_wpt
                )
            )

        if dist_to_wpt <= dist:
            self.current_waypoint += 1
            self.num_cycles = 0
        if self.current_waypoint == len(self.waypoints):
            self.stop(g, text='reached all waypoints')

    def do_abort(self, g):
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return GoToWaypointListResult()
