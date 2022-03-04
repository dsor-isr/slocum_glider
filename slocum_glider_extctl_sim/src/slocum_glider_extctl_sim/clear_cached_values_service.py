import rospy

from std_srvs.srv import Trigger, TriggerResponse


class ClearCachedValuesService:
    def __init__(self):
        self.s = rospy.Service('extctl/clear_cached_values',
                               Trigger,
                               self.handle)

    def handle(self, req):
        return TriggerResponse(success=True)
