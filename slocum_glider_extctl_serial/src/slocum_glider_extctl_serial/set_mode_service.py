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

        # Compute mask to activate (assume modes are 0-indexed)
        if modes_to_activate:
            activate_mask = sum([2**i for i in modes_to_activate])
            rospy.loginfo('Enabling modes with mask %s', activate_mask)
            self.ser.send_message(ensure_binary('MD,{},{}'.format(activate_mask, 255)))

        if modes_to_deactivate:
            # Compute mask to deactivate (assume modes are 0-indexed)
            deactivate_mask = sum([2**i for i in modes_to_deactivate])
            rospy.loginfo('Disabling modes with mask %s', deactivate_mask)
            self.ser.send_message(ensure_binary('MD,{},{}'.format(deactivate_mask, 0)))

        return SetModeResponse(success=True)
