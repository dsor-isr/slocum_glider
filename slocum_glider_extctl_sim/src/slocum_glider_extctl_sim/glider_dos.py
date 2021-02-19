"""Logic for handling gliderdos interactions (used only when a mission is not
running).

"""

import shlex

from .file_parsing import parse_value


class GliderDos(object):
    def __init__(self, g):
        self.g = g
        self.running = bool(self.g.state.x_in_gliderdos)

    def start(self):
        self.g.state.x_in_gliderdos = True
        self.running = True
        self.print_ps1()

    def stop(self):
        self.g.state.x_in_gliderdos = False
        self.running = False

    def print_ps1(self):
        # TODO: Print the previous mission exit code
        self.g.console_writer('GliderDos N >')

    def handle_command(self, command):
        if not self.running:
            raise ValueError('GliderDos is not running!')
        split_command = shlex.split(command)
        if split_command[0] == 'report':
            self.g.report(split_command[1:])
        elif split_command[0] == 'run':
            self.g.run_mission(split_command[1:])
        elif split_command[0] == 'whoru':
            self.g.console_writer('Vehicle Name: ' + self.g.name + '\n')
        elif split_command[0] == 'get':
            name = split_command[1]
            value = self.g.state[name]
            self.g.console_writer('sensor: ' + name + ' = ' + str(value) + ' '
                                  + self.g.masterdata.sensors[name].units)
        elif split_command[0] == 'put':
            name = split_command[1]
            value = split_command[2]
            units = self.g.masterdata.sensors[name].units
            self.g.state[name] = parse_value(units, value)
        else:
            self.g.log('Unknown command: ' + split_command[0])
        self.print_ps1()
