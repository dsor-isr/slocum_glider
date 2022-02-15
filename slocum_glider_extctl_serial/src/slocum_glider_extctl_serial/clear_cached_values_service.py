import rospy

from std_srvs.srv import Trigger, TriggerResponse


class ClearCachedValuesService:
    def __init__(self, ser):
        self.s = rospy.Service('extctl/clear_cached_values',
                               Trigger,
                               self.handle)
        self.ser = ser

    def handle(self, req):
        self.ser.clear_cached_values()
        return TriggerResponse(success=True)
