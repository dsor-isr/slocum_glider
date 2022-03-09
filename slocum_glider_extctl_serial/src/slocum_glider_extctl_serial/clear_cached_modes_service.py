import rospy

from std_srvs.srv import Trigger, TriggerResponse


class ClearCachedModesService:
    def __init__(self, ser):
        self.s = rospy.Service('extctl/clear_cached_modes',
                               Trigger,
                               self.handle)
        self.ser = ser

    def handle(self, req):
        self.ser.clear_cached_modes()
        return TriggerResponse(success=True)
