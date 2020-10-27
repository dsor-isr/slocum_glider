from datetime import datetime
from math import trunc

from .behavior import (BEHAVIOR_STATE_FINISHED,
                       BEHAVIOR_STATE_MISSION_COMPLETE,
                       BEHAVIOR_STATE_UNINITED, BehaviorWithSubstates,
                       Substate)
from .climb_to import ClimbToBehavior
from ..modes import (BPUMP_MODE_ABSOLUTE, PITCH_MODE_BATT_POS,
                     THRUSTER_MODE_POWER)


def stay_at_surface_control(x):
    x.cc_bpump_mode = BPUMP_MODE_ABSOLUTE
    x.cc_bpump_value = 260.0
    x.cc_pitch_mode = PITCH_MODE_BATT_POS
    x.cc_pitch_value = 0.738
    x.cc_thruster_mode = THRUSTER_MODE_POWER
    x.cc_thruster_value = 0


def print_surface_dialog(g):
    c = g.console_writer
    c('\n')
    c('\n')
    c('\n')
    c('Glider ' + g.name + ' at surface.\n')
    # TODO: Add reason
    # TODO: Add MissionNum to following line
    c('MissionName:' + g.active_mission.name + '\n')
    c('Vehicle Name: ' + g.name + '\n')
    curr_time = datetime.now().strftime('%c')
    mt = trunc(g.state.m_present_secs_into_mission)
    c('Curr Time: {} MT: {:>7}\n'.format(curr_time, mt))
    # TODO: Add location info, sensors, devices, and abort history. Note that
    # to fully replicate what the glider prints, we likely need to also record
    # when sensors are modified in the state object.

    # TODO: Add Control-R
    c('   Hit Control-C to END    the mission, i.e. GliderDos\n')
    # TODO: Add Ctrl-E, Ctrl-W, Ctrl-F, S, !, Ctrl-T
    c('\n')
    c('\n')
    c('\n')


class SurfaceBehavior(BehaviorWithSubstates):
    NAME = 'surface'
    HAS_WAITING_STATE = True

    class StayAtSurface(Substate):

        def compute_controls(self, x):
            stay_at_surface_control(x)

    class Uninited(Substate):
        DESCRIPTION = 'UnInited'

        def compute_controls(self, x):
            # Ensure no other surface action can run.
            x.x_surface_active = self.parent_behavior.index

        def next_state(self, parent, x):
            return parent.ClimbTo(parent)

    class ClimbTo(Substate):
        DESCRIPTION = 'climb_to the surface'

        def init(self):
            self.parent_behavior.climb_behavior.set_state(BEHAVIOR_STATE_UNINITED)  # noqa: E501

        def compute_controls(self, x):
            self.parent_behavior.climb_behavior.step(x)

        def next_state(self, parent, x):
            if parent.climb_behavior.state == BEHAVIOR_STATE_FINISHED:
                return parent.WaitingVarious(parent)

    class WaitingVarious(StayAtSurface):
        DESCRIPTION = 'Waiting for various sensors'

        def next_state(self, parent, x):
            return parent.WaitingGPS(parent)

    class WaitingGPS(StayAtSurface):
        DESCRIPTION = 'Waiting for GPS fix'

        def next_state(self, parent, x):
            if x.m_gps_full_status == 0:
                return parent.PickingComms(parent)

    class PickingComms(StayAtSurface):
        DESCRIPTION = 'Picking iridium or freewave'

        def next_state(self, parent, x):
            return parent.WaitingMoreGPS(parent)

    class WaitingMoreGPS(StayAtSurface):
        DESCRIPTION = 'Waiting for more gps fixes'

        def next_state(self, parent, x):
            if x.m_gps_full_status == 0:
                return parent.WaitingForCtrlC(parent)

    class WaitingForCtrlC(StayAtSurface):
        DESCRIPTION = 'Waiting for control-C to exit/resume'

        def init(self):
            self.count = 0

        def compute_controls(self, x):
            # TODO: Actually implement the surface dialog
            self.x_in_surface_dialog = 1 << (self.parent_behavior.index - 1)
            if self.count == 0:
                print_surface_dialog(self.parent_behavior.g)
            self.count += 1

        def next_state(self, parent, x):
            line = parent.g.pop_pending_line()
            if line == 'Ctrl-C':
                parent.terminate_mission = True
                self.x_in_surface_dialog = 0
                # TODO: This transition is yet to be observed.
                return parent.AllDone(parent)

    # TODO: These transitions need to be coded
    # 7 -> 9 Turning on thruster burst
    # 9 -> 10 Waiting for final gps fix
    # 10 -> 13 All done

    class ThrusterBurst(StayAtSurface):
        DESCRIPTION = 'Turning on thruster burst'

    class WaitingForFinalGPS(StayAtSurface):
        DESCRIPTION = 'Waiting for final gps fix'

    class AllDone(Substate):
        DESCRIPTION = 'All done'
        TERMINAL = True

        def compute_controls(self, x):
            # Let other surfacing actions run
            x.x_surface_active = 0

    # 1 climb_to the surface
    # 2 Waiting for various sensors
    # 3 Waiting for GPS fix
    # 4 Picking iridium or freewave
    # 5 Waiting for more gps fixes
    # 7 Waiting for control-C to exit/resume

    SUBSTATES = [Uninited, ClimbTo, WaitingVarious, WaitingGPS, PickingComms,
                 WaitingMoreGPS, None, WaitingForCtrlC, None, ThrusterBurst,
                 WaitingForFinalGPS, None, None, AllDone]

    def should_start(self, x):
        # Start only if no other surface actions are running.
        return ((x.x_surface_active == 0)
                and super(SurfaceBehavior, self).should_start(x))

    def init(self):
        self.terminate_mission = False
        args = self.args

        # TODO: Handle c_stop_when_air_pump
        climb_args = {
            'target_depth': 4,
            'target_altitude': -1,
            'use_bpump': args.c_use_bpump,
            'bpump_value': args.c_bpump_value,
            'use_pitch': args.c_use_pitch,
            'pitch_value': args.c_pitch_value,
            'start_when': 0,
            'use_thruster': args.c_use_thruster,
            'thruster_value': args.c_thruster_value
        }

        self.climb_behavior = ClimbToBehavior(climb_args,
                                              self.index * 100 + 1,
                                              self.g)

    def stop_state(self):
        if self.terminate_mission:
            return BEHAVIOR_STATE_MISSION_COMPLETE
        else:
            return super(SurfaceBehavior, self).stop_state()
