from .behavior import BehaviorWithSubstates, Substate, maybe_deref


class DiveToBehavior(BehaviorWithSubstates):
    class Uninited(Substate):
        DESCRIPTION = 'UnInited'

        def next_state(self, parent, x):
            return parent.Waiting(parent)

    class Waiting(Substate):
        DESCRIPTION = 'waiting for initial depth reading'

        def next_state(self, parent, x):
            if x.m_depth_rejected != 2:
                if x.m_depth < self.args.target_depth:
                    return parent.Starting(parent)
                else:
                    return parent.Complete(parent)

    class Starting(Substate):
        DESCRIPTION = 'Starting the dive'

        def compute_controls(self, x):
            x.c_pinger_on = 10

        def next_state(self, parent, x):
            return parent.Diving(parent)

    class Diving(Substate):
        DESCRIPTION = 'diving'

        def compute_controls(self, x):
            use_bpump = self.args.use_bpump
            bpump_value = maybe_deref(self.args.bpump_value, x)
            use_pitch = self.args.use_pitch
            pitch_value = maybe_deref(self.args.pitch_value, x)
            use_thruster = self.args.use_thruster
            thruster_value = maybe_deref(self.args.thruster_value, x)

            x.cc_bpump_mode = use_bpump
            x.cc_bpump_value = bpump_value
            x.cc_pitch_mode = use_pitch
            x.cc_pitch_value = pitch_value
            x.cc_thruster_mode = use_thruster
            x.cc_thruster_value = thruster_value

            # u.c_ballast_pumped = min(max(bpump_value,
            #                          -x.x_ballast_pumped_max),
            #                          x.x_ballast_pumped_max)

        def next_state(self, parent, x):
            if x.m_depth >= self.args.target_depth:
                return parent.Complete(parent)

    class Complete(Substate):
        DESCRIPTION = 'Complete reached depth'
        TERMINAL = True

    NAME = 'dive_to'

    SUBSTATES = [Uninited, Waiting, None, Starting, Diving, Complete]
