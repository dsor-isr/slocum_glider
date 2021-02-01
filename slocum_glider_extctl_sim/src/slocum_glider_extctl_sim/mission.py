"""Parsing mission files"""

from .behaviors import make_behavior, stack_is_idle
from .file_parsing import parse_value, split_line


class MissionSensor:
    def __init__(self, name, units, default_value):
        self.name = name
        self.units = units
        self.default_value = default_value


class Mission:
    def __init__(self, sensors, behaviors, glider_name, installed):
        self.sensors = sensors
        self.behaviors = behaviors
        self.glider_name = glider_name
        self.installed = installed

    def stack_is_idle(self, x):
        return stack_is_idle(x)


def make_mission(mi_file, g):
    sensors = {}
    behaviors = []
    this_b_name = None
    this_b_args = None
    b_index = 1
    installed = []
    glider_name = None

    for raw_line in mi_file:
        line = split_line(raw_line)
        if not line:
            continue
        if line[0] == 'name':
            # Used in autoexec.mi to set the name of the glider.
            glider_name = line[1]
        elif line[0] == 'installed':
            # Used in autoexec.mi to specify what instruments are installed.
            installed.append(line[1])
        elif line[0] == "sensor:":
            name_and_units = line[1].split("(")
            name = name_and_units[0]
            units = name_and_units[1][:-1]
            if len(line) > 2:
                default = parse_value(units, line[2])
            else:
                default = None
            sensors[name] = MissionSensor(name, units, default)

        elif line[0] == "behavior:":
            if this_b_name:
                behaviors.append(make_behavior(this_b_name,
                                               this_b_args,
                                               b_index,
                                               g))
                b_index += 1
            this_b_args = {}
            this_b_name = line[1]

        elif line[0] == "b_arg:":
            name_and_units = line[1].split("(")
            name = name_and_units[0]
            units = name_and_units[1][:-1]
            value = parse_value(units, line[2])
            this_b_args[name] = value
        else:
            raise ValueError('Cannot parse mission line: ' + raw_line)

    if this_b_name:
        behaviors.append(make_behavior(this_b_name,
                                       this_b_args,
                                       b_index,
                                       g))

    return Mission(sensors, behaviors, glider_name, installed)
