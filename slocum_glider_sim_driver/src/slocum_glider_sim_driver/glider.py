"""Implementation of the glider control system, proglets, etc."""

from contextlib import closing
from math import trunc

import rospy
from six import itervalues

from .behaviors import BEHAVIOR_STATE_MISSION_COMPLETE
from .extctl import ExtctlProglet
from .glider_dos import GliderDos
from .lmc import latlon_to_lmc, set_lmc_origin
from .masterdata import parse_masterdata_file
from .mission import make_mission
from .modes import (BPUMP_MODE_ABSOLUTE, MODE_IGNORE, MODE_UNSET,
                    PITCH_MODE_BATT_POS, PITCH_MODE_PITCH_ONCE,
                    PITCH_MODE_PITCH_SERVO)
from .state import GliderState


class Glider:
    """Represents the simulated glider control system."""

    def __init__(self, glider_fs, science_fs,
                 masterdata_file_path,
                 command_cb=None,
                 update_state_cb=None,
                 console_writer=None):
        with open(masterdata_file_path) as f:
            self.masterdata = parse_masterdata_file(f)
        self.state = GliderState(self.masterdata)

        self.glider_fs = glider_fs
        self.science_fs = science_fs
        self.stop_flag = False
        self.command_cb = command_cb
        self.update_state_cb = update_state_cb
        self.console_writer = console_writer
        self.pending_lines = []

        self.last_log_cycle = None
        self.active_mission = None

        # Run autoexec.mi
        with closing(self.open_flight_file(['config', 'autoexec.mi'])) as f:
            autoexec_string = f.read()
        mission = make_mission(autoexec_string.splitlines(),
                               self)
        self.name = mission.glider_name
        self.installed = mission.installed
        for sensor in itervalues(mission.sensors):
            self.state[sensor.name] = sensor.default_value

        self.compute_init_values()

        # Create the proglets
        self.extctl = ExtctlProglet(self)

        # Start GliderDos!
        self.glider_dos = GliderDos(self)
        self.glider_dos.start()

    def compute_init_values(self):
        state = self.state
        # Compute x_battpos_max:
        state.x_battpos_max = (state.f_battpos_safety_max
                               - state.f_battpos_deadzone_width)

        # HACK: Need to figure out how real glider hardware determines the
        # starting state for everything. For now, just pitch nose down and stay
        # at the surface.
        state.dc_c_battpos = min(0.1, state.x_battpos_max)
        state.dc_c_oil_volume = state.x_ballast_pumped_max

    def compute_lmc_position(self):
        """Translate the current coordinates of the glider into LMC."""
        state = self.state
        state.m_x_lmc, state.m_y_lmc = latlon_to_lmc(state.m_lat,
                                                     state.m_lon,
                                                     state)

    def dynamic_control(self):
        """Run dynamic control. Takes the output of layered control and determines the
        actual low level controls.

        """
        start_time = rospy.get_time()
        state = self.state

        # Compute the desired oil volume
        if state.cc_final_bpump_mode == BPUMP_MODE_ABSOLUTE:
            final_bpump_value = state.cc_final_bpump_value
            state.dc_c_oil_volume = max(min(final_bpump_value,
                                            state.x_ballast_pumped_max),
                                        -state.x_ballast_pumped_max)
        elif state.cc_final_pitch_mode == MODE_IGNORE:
            pass
        elif state.cc_final_bpump_mode == MODE_UNSET:
            pass
        else:
            raise ValueError('Cannot yet handle bpump mode '
                             + str(state.cc_final_bpump_mode))

        # Compute the desired battery position.
        if state.cc_final_pitch_mode == PITCH_MODE_BATT_POS:
            state.dc_c_battpos = max(min(state.cc_final_pitch_value,
                                         state.x_battpos_max),
                                     -state.x_battpos_max)
        elif state.cc_final_pitch_mode == MODE_IGNORE:
            pass
        elif state.cc_final_pitch_mode == MODE_UNSET:
            pass
        elif (state.cc_final_pitch_mode == PITCH_MODE_PITCH_ONCE
              or state.cc_final_pitch_mode == PITCH_MODE_PITCH_SERVO):
            # TODO: Potentially compute battery position. For now, the
            # simulator just jumps to our desired pitch.
            pass
        else:
            raise ValueError('Cannot yet handle pitch mode '
                             + str(state.cc_final_pitch_mode))

        # TODO: Handle heading once simulator supports it.
        end_time = rospy.get_time()
        self.state.x_dc_time = (end_time - start_time) * 1000

    def layered_control(self):
        """Run layered control."""

        start_time = rospy.get_time()

        # First, clear out all controls from previous cycle.
        state = self.state
        for mode_name in ['heading', 'pitch', 'bpump', 'thruster']:
            state['cc_' + mode_name + '_mode'] = MODE_UNSET
            state['cc_' + mode_name + '_value'] = 0
        for mode_name in ['threng', 'inflection', 'depth_state']:
            state['cc_' + mode_name + '_mode'] = MODE_UNSET
        state.cc_mission_status_mode = -3
        state.cc_is_comatose = 0
        state.cc_time_til_inflect = -1
        state.cc_behavior_state = MODE_UNSET

        mission = self.active_mission
        mission_complete = False
        if mission:
            for b in reversed(mission.behaviors):
                b.step(self.state)
                if b.state == BEHAVIOR_STATE_MISSION_COMPLETE:
                    mission_complete = True
                    break

        # Update all the final values
        for mode_name in ['heading', 'pitch', 'bpump', 'thruster']:
            state['cc_final_' + mode_name + '_mode'] = state['cc_' + mode_name + '_mode']  # NOQA: E501
            state['cc_final_' + mode_name + '_value'] = state['cc_' + mode_name + '_value']  # NOQA: E501
        for mode_name in ['threng', 'inflection', 'depth_state']:
            state['cc_final_' + mode_name + '_mode'] = state['cc_' + mode_name + '_mode']  # NOQA: E501
        state.cc_final_mission_status_mode = state.cc_mission_status_mode
        state.cc_final_is_comatose = state.cc_is_comatose
        state.cc_final_time_til_inflect = state.cc_time_til_inflect
        state.cc_final_behavior_state = state.cc_behavior_state

        if mission_complete:
            self.active_mission = None
        end_time = rospy.get_time()
        self.state.x_lc_time = (end_time - start_time) * 1000

    def update_sensors(self):
        """Read all sensors from simulated hardware and update their values in the
        control system.

        """
        start_time = rospy.get_time()
        cb = self.update_state_cb
        if cb:
            cb(self)
        end_time = rospy.get_time()
        self.state.x_sp_time = end_time - start_time

    def run_one_cycle(self):
        start_time = rospy.get_time()
        state = self.state
        state.m_present_secs_into_mission = (start_time -
                                             state.m_mission_start_time)
        state.m_present_time = start_time
        state.m_cycle_number += 1

        self.update_sensors()
        if self.active_mission:
            self.compute_lmc_position()
        self.layered_control()
        self.dynamic_control()

        # Run the proglets
        self.extctl.send_sensor_values()

        # Send the control message!
        if self.command_cb:
            self.command_cb(self)

        # Reenable glider dos if needed
        if not self.active_mission and not self.glider_dos.running:
            self.glider_dos.start()

    def run(self):
        """Run the control loops until asked to stop."""
        while not self.stop_flag and not rospy.is_shutdown():
            start_time = rospy.get_time()

            self.run_one_cycle()

            end_time = rospy.get_time()
            duration = end_time - start_time
            sleep_time = self.state.u_cycle_time - duration
            if sleep_time > 0:
                rospy.sleep(sleep_time)

    def report_change(self, name, old, new):
        """Function to report a change in a sensor."""
        self.log('sensor: ' + name + ' = ' + str(new) + ' '
                 + self.masterdata.sensors[name].units)

    def report(self, args):
        """Set up reports for sensor changes/updates."""
        if args[0] == 'all':
            for sensor in itervalues(self.masterdata.sensors):
                self.state.add_change_listener(sensor.name, self.report_change)
        elif args[0] == 'clearall':
            for sensor in itervalues(self.masterdata.sensors):
                self.state.remove_change_listener(sensor.name,
                                                  self.report_change)
                self.state.remove_update_listener(sensor.name,
                                                  self.report_change)
        elif args[0] == '+':
            for sensor in args[1:]:
                self.state.add_change_listener(sensor, self.report_change)
        elif args[0] == '++':
            for sensor in args[1:]:
                self.state.add_update_listener(sensor, self.report_change)
        elif args[0] == '-':
            for sensor in args[1:]:
                self.state.remove_change_listener(sensor,
                                                  self.report_change)
                self.state.remove_update_listener(sensor,
                                                  self.report_change)
        else:
            self.log('Unknown argument ' + args[0])

    def run_mission(self, args):
        with closing(self.open_flight_file(['missions', args[0]])) as mi_file:
            mission_string = mi_file.read()
        self.glider_dos.stop()
        mission = make_mission(mission_string.splitlines(),
                               self)
        mission.name = args[0]
        set_lmc_origin(self.state)
        self.active_mission = mission
        self.state.m_mission_start_time = rospy.get_time()

    def cmd(self, command):
        self.console_writer(command + '\n')
        if self.state.x_in_gliderdos:
            self.glider_dos.handle_command(command)
        else:
            self.pending_lines.append(command)
            # TODO: Cap the size of pending_lines?

    def pop_pending_line(self):
        if len(self.pending_lines) > 0:
            return self.pending_lines.pop(0)
        else:
            return None

    def open_flight_file(self, path, flags='r'):
        """Open a file on the flight computer."""
        return self.glider_fs.open(path, flags)

    def open_science_file(self, path, flags='r'):
        """Open a file on the science computer."""
        return self.science_fs.open(path, flags)

    def log(self, msg):
        """Log a message, prefixing each line with the mission time and cycle if
        necessary.

        """
        if self.console_writer:
            prefix = ''
            state = self.state
            truncated_time = state.m_present_secs_into_mission % 1000000

            if truncated_time < 999:
                prefix = '{:>6.2f}'.format(truncated_time)
            else:
                prefix = '{:>6d}'.format(trunc(truncated_time))

            if self.last_log_cycle != state.m_cycle_number:
                self.last_log_cycle = state.m_cycle_number
                prefix = prefix + ' {:>2d} '.format(int(state.m_cycle_number
                                                        % 100))
            else:
                prefix = prefix + '    '
            for line in msg.splitlines():
                self.console_writer(prefix + line + '\n')
