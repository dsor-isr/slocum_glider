import rospy

from .blocker import block_until_start
from .extctl import Extctl


class BackseatInterface:
    def __init__(self):
        self.extctl_interface = None

    def start(self):
        if rospy.get_param('~extctl/enabled'):
            serial_port_name = rospy.get_param('~serial_port/device')
            block_until_start(serial_port_name)
            self.extctl_interface = Extctl()
            self.extctl_interface.start()

    def stop(self):
        if self.extctl_interface:
            self.extctl_interface.stop()
