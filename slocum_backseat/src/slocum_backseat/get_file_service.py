import rospy
from slocum_msgs.srv import GetFile, GetFileResponse


class GetFileService:
    def __init__(self, ser):
        self.s = rospy.Service('extctl/get_file',
                               GetFile,
                               self.get_file)

    def get_file(self, req):
        pass

    def handle_serial_msg(self, msg):
        """Called when a sentence of type FI is received over the serial port.

        msg is a string starting with "FI," followed by up to 256 bytes of
        base64 encoded data.

        """
        pass
