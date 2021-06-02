from .behavior import BEHAVIOR_STATE_FINISHED, BEHAVIOR_STATE_WAITING, Behavior
from .goto_wpt import GotoWptBehavior


INITIAL_WPT_CLOSEST = -2
INITIAL_WPT_NEXT = -1


class GotoListBehavior(Behavior):
    NAME = 'goto_list'

    def init(self):
        super(GotoListBehavior, self).init()
        sub = []
        for i in range(int(self.args.num_waypoints)):
            sub_args = {
                'start_when': 0,
                'stop_when': self.args.list_stop_when,
                'when_wpt_dist': self.args.list_when_wpt_dist,
                'wpt_units':  self.args['wpt_units_%d' % i],
                'wpt_x': self.args['wpt_x_%d' % i],
                'wpt_y': self.args['wpt_y_%d' % i],
                'end_action': 0
            }
            sub.append(GotoWptBehavior(sub_args, self.index * 100 + i, self.g))
            sub[i].init()
            sub[i].set_state(BEHAVIOR_STATE_WAITING)
        self.sub_behaviors = sub
        self.active_waypoint = None
        self.num_waypoints_done = 0

    def compute_initial_waypoint(self, x):
        # We need to figure out which waypoint we're going to go for first!
        if self.args.initial_wpt == INITIAL_WPT_CLOSEST:
            closest_index = -1
            closest_distance = float("Inf")
            index = 0
            for sub in self.sub_behaviors:
                dist = sub.distance_from_waypoint(x)
                if dist < closest_distance:
                    closest_distance = dist
                    closest_index = index
                index += 1
            return closest_index
        elif self.args.initial_wpt == INITIAL_WPT_NEXT:
            raise NotImplementedError('Unclear how to implement this')
        else:
            return int(self.args.initial_wpt)

    def compute_controls(self, x):
        if self.active_waypoint is None:
            self.active_waypoint = self.compute_initial_waypoint(x)

        sub = self.sub_behaviors[self.active_waypoint]

        # This used to immediately start the next goto in the same control
        # cycle that the previous one finished. However, that didn't give time
        # for x_hit_a_waypoint to propagate.
        sub.step(x)
        if sub.state == BEHAVIOR_STATE_FINISHED:
            sub.set_state(BEHAVIOR_STATE_WAITING)
            self.num_waypoints_done += 1
            self.active_waypoint += 1
            self.active_waypoint = (self.active_waypoint
                                    % int(self.args.num_waypoints))

    def should_stop(self, x):
        num_legs_to_run = self.args.num_legs_to_run
        if num_legs_to_run == -2:
            return (self.num_waypoints_done > 0
                    and self.active_waypoint == 0)
        elif num_legs_to_run == -1:
            return False
        else:
            return self.num_waypoints_done >= num_legs_to_run
