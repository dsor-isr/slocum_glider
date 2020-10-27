from math import sqrt

from .behavior import Behavior
from .goto_wpt import GotoWptBehavior, WPT_UNITS_LAT_LONG


INITIAL_WPT_CLOSEST = -2
INITIAL_WPT_NEXT = -1


class GotoListBehavior(Behavior):
    NAME = 'goto_list'
    CONTROLS = ['heading']

    def init(self, g):
        sub = []
        for i in range(self.args.num_waypoints):
            sub_args = {
                'start_when': 0,
                'stop_when': self.args.list_stop_when,
                'when_wpt_dist': self.args.when_wpt_dist,
                'wpt_units': WPT_UNITS_LAT_LONG,
                'wpt_x': getattr(self.args, 'wpt_x_%d' % i),
                'wpt_y': getattr(self.args, 'wpt_y_%d' % i),
                'end_action': 0
            }
            sub.append(GotoWptBehavior(sub_args, self.index + i * 0.1, self.g))
            sub[i].init(g)
        self.sub_behaviors = sub
        self.active_waypoint = None

    def compute_controls(self, u, g):
        if self.active_waypoint is None:
            # We need to figure out which waypoint we're going to go for first!
            if self.args.initial_wpt == INITIAL_WPT_CLOSEST:
                closest_index = -1
                closest_distance = float("Inf")
                index = 0
                for sub in self.sub_behaviors:
                    dist = sqrt((g.m_x_lmc - sub.wpt_x_lmc)**2
                                + (g.m_y_lmc - sub.wpt_y_lmc)**2)
                    if dist < closest_distance:
                        closest_distance = dist
                        closest_index = index
                    index += 1
                self.active_waypoint = closest_index
            elif self.args.initial_wpt == INITIAL_WPT_NEXT:
                self.active_waypoint = self.last_waypoint + 1
                if self.active_waypoint == 8:
                    self.active_waypoint = 0
            else:
                self.active_waypoint = self.args.initial_wpt

        # We know the active waypoint. Defer to it for controls:
        self.sub_behaviors[self.active_waypoint].compute_controls(u, g)
