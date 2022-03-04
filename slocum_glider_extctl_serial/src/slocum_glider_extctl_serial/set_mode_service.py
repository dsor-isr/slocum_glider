import rospy

from six import ensure_binary
from slocum_glider_msgs.srv import SetMode, SetModeResponse


class SetModeService:
    def __init__(self, ser):
        self.s = rospy.Service('extctl/set_mode',
                               SetMode,
                               self.set_mode)
        # Save a reference to the serial port object
        self.ser = ser

    def set_mode(self, req):

        modes_to_activate = req.modes_to_activate
        modes_to_deactivate = req.modes_to_deactivate

        # Check that the same mode is not being both activated and deactivated
        coll = set(modes_to_activate) & set(modes_to_deactivate)
        if coll:
            return SetModeResponse(success=False, message="Cannot activate & deactivate mode(s): {}".format(list(coll)))

        if modes_to_activate:
            self.ser.activate_modes(modes_to_activate)

        if modes_to_deactivate:
            self.ser.deactivate_modes(modes_to_deactivate)

        self.ser.send_mode_msg()

        return SetModeResponse(success=True)
