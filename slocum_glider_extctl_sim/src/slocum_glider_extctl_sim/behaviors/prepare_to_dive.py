from .behavior import BehaviorWithSubstates, Substate
from ..modes import (BPUMP_MODE_ABSOLUTE, HEADING_MODE_FIN,
                     PITCH_MODE_BATT_POS, THRUSTER_MODE_NONE)


class PrepareToDiveBehavior(BehaviorWithSubstates):
    """Prepare to dive.

    Keeps the glider at the surface with the tail out of the water until there
    is a GPS fix.

    """
    class Uninited(Substate):
        DESCRIPTION = 'UnInited'

        def next_state(self, parent, x):
            return parent.WaitForFix(parent)

    class WaitForFix(Substate):
        DESCRIPTION = 'Waiting for initial GPS fix'

        def compute_controls(self, x):
            x.c_gps_on = 1
            x.c_alt_time = -1
            x.cc_heading_mode = HEADING_MODE_FIN
            # FIXME: Simulator currently chokes on batt pos
            x.cc_pitch_mode = PITCH_MODE_BATT_POS
            # TODO: Taken from nicoya. Need to determine if this is universal.
            x.cc_pitch_value = 0.738
            x.cc_thruster_mode = THRUSTER_MODE_NONE
            x.cc_bpump_mode = BPUMP_MODE_ABSOLUTE
            # TODO: Taken from nicoya. Need to determine if this is universal.
            x.cc_bpump_value = 260
            # TODO: Don't really have documentation for these, need to examine
            # a bit more.
            x.cc_threng_mode = 0
            x.cc_depth_state_mode = 0

        def next_state(self, parent, x):
            if x.m_gps_status == 0:
                return parent.AllDone(parent)

    class AllDone(Substate):
        DESCRIPTION = 'All done'
        TERMINAL = True

        def compute_controls(self, x):
            pass

    NAME = 'prepare_to_dive'
    SUBSTATES = [Uninited, WaitForFix, None, AllDone]
