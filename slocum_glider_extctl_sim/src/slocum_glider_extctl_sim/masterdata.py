"""Parsing masterdata"""

from .file_parsing import parse_value, split_line

from six import itervalues


class MasterdataSensor:
    def __init__(self, name, units, default_value):
        self.name = name
        self.units = units
        self.default_value = default_value


class MasterdataBehaviorArg:
    def __init__(self, name, units, default_value):
        self.name = name
        self.units = units
        self.default_value = default_value


class MasterdataBehavior:
    def __init__(self, name, args):
        self.name = name
        self.args = args


class Masterdata:
    def __init__(self, sensors, behaviors):
        self.sensors = sensors
        self.behaviors = behaviors

    def behavior_default_args(self, behavior_name):
        behavior_def = self.behaviors[behavior_name]
        args = {}

        for arg_def in itervalues(behavior_def.args):
            args[arg_def.name] = arg_def.default_value

        return args


def parse_masterdata_file(f):
    sensors = {}
    behaviors = {}
    this_behavior_args = {}

    for line in f:
        line = split_line(line)
        if not line:
            continue
        if line[0] == "sensor:":
            name_and_units = line[1].split("(")
            name = name_and_units[0]
            units = name_and_units[1][:-1]
            if len(line) > 2:
                default = parse_value(units, line[2])
            else:
                default = None
            sensors[name] = MasterdataSensor(name, units, default)

        elif line[0] == "behavior:":
            this_behavior_args = {}
            behaviors[line[1]] = MasterdataBehavior(line[1],
                                                    this_behavior_args)

        elif line[0] == "b_arg:":
            name_and_units = line[1].split("(")
            name = name_and_units[0]
            units = name_and_units[1][:-1]
            default = parse_value(units, line[2])
            this_behavior_args[name] = MasterdataBehaviorArg(name, units,
                                                             default)
    # Fixup the masterdata.
    #
    # There is a mismatch between masterdata.mi and implementation. The
    # masterdata file says goto_list's wpt_units_N defaults to 0. However, in
    # practice it seems that it defaults to 2. It's also questionable whether
    # or not anything other than 2 can be set. We perform the fixup here
    # (instead of in the masterdata.mi) because if we ever update the
    # masterdata, we want to avoid non-obvious bugs from forgetting to make the
    # change.

    for i in range(8):
        behaviors['goto_list'].args['wpt_units_%d' % i].default_value = 2

    return Masterdata(sensors, behaviors)
