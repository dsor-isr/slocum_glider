from .behavior import BehaviorWithSubstates, Substate


def is_diving(x):
    return x.cc_bpump_value < 0


def is_climbing(x):
    return x.cc_bpump_value > 0


def is_on_surface(x):
    return False


class SampleBehavior(BehaviorWithSubstates):
    """Controls the sampling of science sensors.

    Currently just a dummy behavior.

    """

    class InnerSubstate(Substate):
        def next_state(self, parent, x):
            if is_diving(x):
                next_class = parent.Diving
            elif is_climbing(x):
                next_class = parent.Climbing
            elif is_on_surface(x):
                next_class = parent.OnSurface
            else:
                next_class = parent.InLimbo

            if next_class != self.__class__:
                return next_class(parent)

    class Uninited(InnerSubstate):
        DESCRIPTION = 'UnInited'

    class Diving(InnerSubstate):
        DESCRIPTION = 'Diving'

    class Climbing(InnerSubstate):
        DESCRIPTION = 'Climbing'

    class OnSurface(InnerSubstate):
        DESCRIPTION = 'On Surface'

    class InLimbo(InnerSubstate):
        DESCRIPTION = 'In Limbo'

    NAME = 'sample'
    SUBSTATES = [Uninited, Diving, None, Climbing, OnSurface, InLimbo]
