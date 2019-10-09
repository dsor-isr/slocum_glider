import rospy

from slocum_glider_msgs.srv import SetString, SetStringResponse


class SetStringService:
    def __init__(self, ser):
        self.s = rospy.Service('extctl/set_string',
                               SetString,
                               self.set_string)
        # Save a reference to the serial port object
        self.ser = ser

    def set_string(self, req):
        "Send raw serial messages"
        self.ser.send_message(req.message)
        return SetStringResponse(success=True)
