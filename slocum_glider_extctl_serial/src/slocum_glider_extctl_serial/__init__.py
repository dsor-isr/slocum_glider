import rospy

from .extctl import Extctl


class BackseatInterface:
    def __init__(self):
        if rospy.get_param('~extctl/enabled'):
            self.extctl_interface = Extctl()
        else:
            self.extctl_interface = None

    def start(self):
        if self.extctl_interface:
            self.extctl_interface.start()

    def stop(self):
        if self.extctl_interface:
            self.extctl_interface.stop()
