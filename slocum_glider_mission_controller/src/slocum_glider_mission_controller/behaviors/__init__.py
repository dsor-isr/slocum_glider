from .follow_bottom import FollowBottomBehavior
from .go_to_waypoint import GoToWaypointBehavior
from .go_to_waypoint_list import GoToWaypointListBehavior
from .stay_at_waypoint import StayAtWaypointBehavior
from .surface import SurfaceBehavior
from .thruster_constant_power import ThrusterConstantPowerBehavior
from .thruster_off import ThrusterOffBehavior
from .yo import YoBehavior


BEHAVIOR_CLASSES = [FollowBottomBehavior, GoToWaypointBehavior,
                    GoToWaypointListBehavior, StayAtWaypointBehavior,
                    SurfaceBehavior, ThrusterConstantPowerBehavior,
                    ThrusterOffBehavior, YoBehavior]

BEHAVIOR_CLASS_MAP = {cls.ACTION_NAME: cls for cls in BEHAVIOR_CLASSES}


__all__ = (['BEHAVIOR_CLASSES', 'behavior_class_for_name']
           + [cls.__name__ for cls in BEHAVIOR_CLASSES])


def behavior_class_for_name(name):
    return BEHAVIOR_CLASS_MAP[name]
