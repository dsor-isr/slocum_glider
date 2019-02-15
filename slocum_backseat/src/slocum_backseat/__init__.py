from extctl import Extctl


class BackseatInterface:
    def __init__(self):
        self.extctl_interface = Extctl()

    def start(self):
        self.extctl_interface.start()

    def stop(self):
        self.extctl_interface.stop()
