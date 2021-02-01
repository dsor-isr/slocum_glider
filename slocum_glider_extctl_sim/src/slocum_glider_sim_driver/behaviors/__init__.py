from .behavior import (BEHAVIOR_STATE_MISSION_COMPLETE,
                       BEHAVIOR_STATE_UNINITED, BEHAVIOR_STATE_WAITING,
                       BEHAVIOR_STATE_ACTIVE, BEHAVIOR_STATE_FINISHED,
                       stack_is_idle)
from .abend import AbendBehavior
from .climb_to import ClimbToBehavior
from .dive_to import DiveToBehavior
from .goto_list import GotoListBehavior
from .goto_wpt import GotoWptBehavior
from .pinger_on import PingerOnBehavior
from .prepare_to_dive import PrepareToDiveBehavior
from .sample import SampleBehavior
from .sensors_in import SensorsInBehavior
from .set_heading import SetHeadingBehavior
from .surface import SurfaceBehavior
from .yo import YoBehavior


__all__ = ['BEHAVIOR_STATE_MISSION_COMPLETE', 'BEHAVIOR_STATE_UNINITED',
           'BEHAVIOR_STATE_WAITING', 'BEHAVIOR_STATE_ACTIVE',
           'BEHAVIOR_STATE_FINISHED', 'make_behavior', 'stack_is_idle']


ALL_BEHAVIORS = [AbendBehavior, ClimbToBehavior, DiveToBehavior,
                 GotoListBehavior, GotoWptBehavior, PingerOnBehavior,
                 PrepareToDiveBehavior, SampleBehavior, SensorsInBehavior,
                 SetHeadingBehavior, SurfaceBehavior, YoBehavior]


def make_behavior(name, args, index, g):
    for b in ALL_BEHAVIORS:
        if name == b.NAME:
            return b(args, index, g)

    raise ValueError('Unknown behavior ' + name)
