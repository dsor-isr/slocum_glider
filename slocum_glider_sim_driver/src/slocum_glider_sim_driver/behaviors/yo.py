from .behavior import (BEHAVIOR_STATE_UNINITED, BEHAVIOR_STATE_FINISHED,
                       Behavior)
from .climb_to import ClimbToBehavior
from .dive_to import DiveToBehavior


class YoBehavior(Behavior):
    """Yo

    Uses the dive_to and climb_to behaviors to make the glider yo.

    """
    NAME = 'yo'

    def init(self):
        super(YoBehavior, self).init()
        args = self.args
        self.yos_done = 0

        # TODO Make sure initial_inflection = 1 is universally OK.
        dive_args = {
            'target_depth': args.d_target_depth,
            'target_altitude': args.d_target_altitude,
            'use_bpump': args.d_use_bpump,
            'bpump_value': args.d_bpump_value,
            'use_pitch': args.d_use_pitch,
            'pitch_value': args.d_pitch_value,
            'start_when': 0,
            'stop_when_hover_for': args.d_stop_when_hover_for,
            'stop_when_stalled_for': args.d_stop_when_stalled_for,
            'speed_min': args.d_speed_min,
            'speed_max': args.d_speed_max,
            'use_thruster': args.d_use_thruster,
            'thruster_value': args.d_thruster_value,
            'depth_rate_method': args.d_depth_rate_method,
            'wait_for_pitch': args.d_wait_for_pitch,
            'wait_for_ballast': args.d_wait_for_ballast,
            'delta_bpump_speed': args.d_delta_bpump_speed,
            'delta_bpump_ballast': args.d_delta_bpump_ballast,
            'time_ratio': args.d_time_ratio,
            'use_sc_model': args.d_use_sc_model,
            'max_thermal_charge_time': args.d_max_thermal_charge_time,
            'max_pumping_charge_time': args.d_max_pumping_charge_time,
            'thr_reqd_pres_mul': args.d_thr_reqd_pres_mul
        }
        self.dive_behavior = DiveToBehavior(dive_args,
                                            self.index * 100 + 1,
                                            self.g)

        # TODO Make sure initial_inflection = 1 is universally OK.
        climb_args = {
            'target_depth': args.c_target_depth,
            'target_altitude': args.c_target_altitude,
            'use_bpump': args.c_use_bpump,
            'bpump_value': args.c_bpump_value,
            'use_pitch': args.c_use_pitch,
            'pitch_value': args.c_pitch_value,
            'start_when': 0,
            'stop_when_hover_for': args.c_stop_when_hover_for,
            'stop_when_stalled_for': args.c_stop_when_stalled_for,
            'speed_min': args.c_speed_min,
            'speed_max': args.c_speed_max,
            'use_thruster': args.c_use_thruster,
            'thruster_value': args.c_thruster_value,
            # TODO: Confirm there's no way to control these via the yo
            # behavior.
            # 'depth_rate_method': args.c_depth_rate_method,
            # 'wait_for_pitch': args.c_wait_for_pitch,
            # 'wait_for_ballast': args.c_wait_for_ballast,
            # 'delta_bpump_speed': args.c_delta_bpump_speed,
            # 'delta_bpump_ballast': args.c_delta_bpump_ballast,
            # 'time_ratio': args.c_time_ratio,
            # 'use_sc_model': args.c_use_sc_model,
            # 'max_thermal_charge_time': args.c_max_thermal_charge_time,
            # 'max_pumping_charge_time': args.c_max_pumping_charge_time,
            # 'thr_reqd_pres_mul': args.c_thr_reqd_pres_mul
        }
        self.climb_behavior = ClimbToBehavior(climb_args,
                                              self.index * 100 + 2,
                                              self.g)

        if self.args.start_diving == 0:
            self.active_subbehavior = 'climb'
        elif self.args.start_diving == 1:
            self.active_subbehavior = 'dive'
        else:
            ValueError('Cannot handle start_diving: '
                       + str(self.args.start_diving))

    def compute_controls(self, x):
        x.c_dive_target_depth = self.args.d_target_depth
        x.c_climb_target_depth = self.args.c_target_depth

        if self.active_subbehavior == 'climb':
            self.climb_behavior.step(x)
            if self.climb_behavior.state == BEHAVIOR_STATE_FINISHED:
                self.climb_behavior.set_state(BEHAVIOR_STATE_UNINITED)
                self.active_subbehavior = 'dive'
                self.yos_done += 1
        else:
            self.dive_behavior.step(x)
            if self.dive_behavior.state == BEHAVIOR_STATE_FINISHED:
                self.dive_behavior.set_state(BEHAVIOR_STATE_UNINITED)
                self.active_subbehavior = 'climb'
                self.yos_done += 1

    def should_stop(self, x):
        num_half_cycles_to_do = self.args.num_half_cycles_to_do
        return super(YoBehavior, self).should_stop(x) \
            or ((num_half_cycles_to_do >= 0)
                and (self.yos_done >= num_half_cycles_to_do))
