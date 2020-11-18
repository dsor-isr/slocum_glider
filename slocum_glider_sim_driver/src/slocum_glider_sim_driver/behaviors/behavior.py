from contextlib import closing

from six import iteritems
from six.moves import collections_abc

from ..file_parsing import parse_value, split_line
from ..modes import MODE_UNSET


BEHAVIOR_STATE_UNINITED = 1
BEHAVIOR_STATE_WAITING = 2
BEHAVIOR_STATE_ACTIVE = 3
BEHAVIOR_STATE_FINISHED = 4
BEHAVIOR_STATE_MISSION_COMPLETE = 5


BAW_IMMEDIATELY = 0
BAW_COMPLETE = 0
BAW_STACK_IDLE = 1
BAW_PITCH_IDLE = 2
BAW_HEADING_IDLE = 3
BAW_UPDOWN_IDLE = 4
BAW_NEVER = 5
BAW_WHEN_SECS = 6
BAW_WHEN_WPT_DIST = 7
BAW_WHEN_HIT_WAYPOINT = 8
BAW_EVERY_SECS = 9
BAW_EVERY_SECS_UPDOWN_IDLE = 10
BAW_SCI_SURFACE = 11
BAW_NOCOMM_SECS = 12
BAW_WHEN_UTC_TIME = 13
BAW_HOVER_ACTIVE = 14


END_ACTION_QUIT = 0
END_ACTION_WAIT_FOR_CTRL_C_RESUME = 1
END_ACTION_RESUME = 2
END_ACTION_DRIFT = 3
END_ACTION_WAIT_FOR_CTRL_C_ONCE = 4
END_ACTION_WAIT_FOR_CTRL_C_QUIT = 5


def state_name(state):
    if state == BEHAVIOR_STATE_UNINITED:
        return 'UnInited'
    elif state == BEHAVIOR_STATE_WAITING:
        return 'Waiting for Activation'
    elif state == BEHAVIOR_STATE_ACTIVE:
        return 'Active'
    elif state == BEHAVIOR_STATE_FINISHED:
        return 'Complete'
    elif state == BEHAVIOR_STATE_MISSION_COMPLETE:
        return 'Mission Complete'


def maybe_deref(value, x):
    if value >= 1000000:
        param = 'u_mission_param_' + chr(ord('a') + (value - 1000000))
        return x[param]
    else:
        return value


def parse_ma_file(f):
    args = {}
    in_waypoints = False
    wpt_index = 0
    # This is a rather... permissive way of parsing these files. Consider
    # actually handling <start:b_arg> and such.
    for line in f:
        line = split_line(line)
        if not line:
            continue
        if line[0] == "b_arg:":
            name_and_units = line[1].split("(")
            name = name_and_units[0]
            units = name_and_units[1][:-1]
            value = parse_value(units, line[2])
            args[name] = value
        elif line[0] == "<start:waypoints>":
            in_waypoints = True
        elif line[0] == "<end:waypoints>":
            in_waypoints = False
        elif in_waypoints:
            args['wpt_x_' + str(wpt_index)] = float(line[0])
            args['wpt_y_' + str(wpt_index)] = float(line[1])
            wpt_index += 1
    return args


def stack_is_idle(x):
    """Returns True if the stack is idle. A straightforward reading of the
description of ths condition "When stack is idle (nothing is being commanded)"
would indicate that it should return True if there are no commands. However,
this does not seem to comport with observations of glider behavior. The best
explanation we can come up with is that the stack is actually considered idle
if any of pitch/heading/up_down are not being controlled.

    """
    return (pitch_is_idle(x)
            or heading_is_idle(x)
            or up_down_is_idle(x))


def pitch_is_idle(x):
    return x.cc_pitch_mode == MODE_UNSET


def heading_is_idle(x):
    return x.cc_heading_mode == MODE_UNSET


def up_down_is_idle(x):
    return x.cc_bpump_mode == MODE_UNSET


class BehaviorArgs(collections_abc.Mapping):
    def __init__(self, args):
        self._args = args
        for name, value in iteritems(args):
            setattr(self, name, value)

    def __getitem__(self, item):
        return self._args.__getitem__(item)

    def __iter__(self):
        return self._args.__iter__()

    def __len__(self):
        return self._args.__len__()


class Behavior(object):
    """The base class for all glider behaviors."""

    HAS_WAITING_STATE = True

    def __init__(self, args, index, g):
        self.orig_args = g.masterdata.behavior_default_args(self.NAME)
        self.orig_args.update(args)
        self.args = BehaviorArgs(self.orig_args)
        self.index = index
        self.g = g

        self.time_entered_state = 0
        self.state = BEHAVIOR_STATE_UNINITED

        self.behavior_name = self.NAME + '_' + str(self.index)

    def set_state(self, new_state):
        self.g.log('behavior ' + self.behavior_name + ': STATE '
                   + state_name(self.state) + ' -> ' +
                   state_name(new_state))
        self.time_entered_state = self.g.state.m_present_secs_into_mission
        self.state = new_state

    def init(self):
        assert self.state == BEHAVIOR_STATE_UNINITED

    def start(self):
        assert self.state == BEHAVIOR_STATE_WAITING

    def step(self, x):
        if self.state == BEHAVIOR_STATE_UNINITED:
            self.init()
            if self.HAS_WAITING_STATE:
                self.set_state(BEHAVIOR_STATE_WAITING)
            else:
                self.set_state(BEHAVIOR_STATE_ACTIVE)

        if self.state == BEHAVIOR_STATE_WAITING \
           and self.should_start(x):
            self.set_state(BEHAVIOR_STATE_ACTIVE)

        if self.state == BEHAVIOR_STATE_ACTIVE:
            self.compute_controls(x)
            if self.should_stop(x):
                self.set_state(self.stop_state())

    def stop_state(self):
        if 'end_action' not in self.args:
            return BEHAVIOR_STATE_FINISHED
        end_action = self.args.end_action
        if end_action == END_ACTION_QUIT:
            return BEHAVIOR_STATE_FINISHED
        elif end_action == END_ACTION_RESUME:
            return BEHAVIOR_STATE_WAITING
        else:
            raise ValueError('Cannot handle end_action '
                             + str(end_action)
                             + ' (yet?).')

    def update_ma_args(self):
        orig_args = self.orig_args

        if 'args_from_file' in orig_args and orig_args['args_from_file'] != -1:
            file_name = "%s%02d.ma" % (self.NAME[0:6],
                                       orig_args['args_from_file'])
            file_name = file_name.upper()
            with closing(self.g.open_flight_file(['mafiles', file_name])) as f:
                ma_args = parse_ma_file(f.read().splitlines())
            new_args = orig_args.copy()
            new_args.update(ma_args)
            self.args = BehaviorArgs(new_args)

    def condition_satisfied(self, x, start_or_stop, when):
        if when == BAW_IMMEDIATELY and start_or_stop == 'start':
            return True
        elif when == BAW_STACK_IDLE:
            return stack_is_idle(x)
        elif when == BAW_PITCH_IDLE:
            return pitch_is_idle(x)
        elif when == BAW_HEADING_IDLE:
            return heading_is_idle(x)
        elif when == BAW_UPDOWN_IDLE:
            return up_down_is_idle(x)
        elif when == BAW_NEVER:
            return False
        elif when == BAW_WHEN_SECS and start_or_stop == 'stop':
            return ((x.m_present_secs_into_mission - self.time_entered_state)
                    >= self.args.when_secs)
        elif when == BAW_WHEN_SECS:
            raise NotImplementedError()
        elif when == BAW_WHEN_WPT_DIST:
            return x.m_dist_to_wpt < self.args.when_wpt_dist
        elif when == BAW_WHEN_HIT_WAYPOINT:
            return x.x_hit_a_waypoint
        elif when == BAW_EVERY_SECS:
            raise NotImplementedError()
        elif when == BAW_EVERY_SECS_UPDOWN_IDLE:
            raise NotImplementedError()
        elif when == BAW_SCI_SURFACE:
            return x.sci_wants_surface > 0
        elif when == BAW_NOCOMM_SECS:
            raise NotImplementedError()
        elif when == BAW_WHEN_UTC_TIME:
            raise NotImplementedError()
        elif when == BAW_HOVER_ACTIVE:
            raise NotImplementedError()
        else:
            raise ValueError('Do not know how to handle ' + start_or_stop
                             + ': ' + str(when))

    def should_start(self, x):
        if 'start_when' in self.args:
            start_when = self.args.start_when
            return self.condition_satisfied(x, 'start', start_when)
        else:
            return True

    def should_stop(self, x):
        if 'stop_when' in self.args:
            stop_when = self.args.stop_when
            return self.condition_satisfied(x, 'stop', stop_when)
        else:
            return False


class Substate:
    TERMINAL = False

    def __init__(self, parent_behavior):
        self.parent_behavior = parent_behavior
        self.args = parent_behavior.args
        self.init()

    def init(self):
        pass

    def compute_controls(self, x):
        pass

    def next_state(self, parent, x):
        return None


class BehaviorWithSubstates(Behavior):
    SUBSTATES = None

    def __init__(self, args, index, g):
        super(BehaviorWithSubstates, self).__init__(args, index, g)
        self.substate = self.SUBSTATES[0](self)
        self.init()

    def substate_index(self, substate):
        return self.SUBSTATES.index(substate.__class__)

    def compute_controls(self, x):
        # TODO: Ensure we should call next_state here instead of using
        # self.substate
        # next_state = self.substate.next_state(self, x) or self.substate
        next_state = self.substate
        while next_state:
            self.substate = next_state
            self.substate.compute_controls(x)
            next_state = self.substate.next_state(self, x)
            if next_state:
                self.g.log('behavior ' + self.behavior_name + ': SUBSTATE '
                           + str(self.substate_index(self.substate)) + ' '
                           + self.substate.DESCRIPTION
                           + '->' + str(self.substate_index(next_state))
                           + ' : ' + next_state.DESCRIPTION)

    def should_stop(self, x):
        return self.substate.TERMINAL
