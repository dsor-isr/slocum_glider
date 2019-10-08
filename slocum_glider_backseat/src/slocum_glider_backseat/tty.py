import subprocess as sub


class SerialConsole:
    def __init__(self, serial_port_name):
        self.serial_port_name = serial_port_name

    def run(self):
        # Need to strip off the /dev/ from the port name.
        sub.call(['sudo', 'agetty', '-J', '-t', '60',
                  self.serial_port_name[5:], '9600'])
