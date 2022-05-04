"""Functionality to send commands to the simulated hardware over ROS."""


from frl_vehicle_msgs.msg import UwGliderCommand
import rospy
from std_msgs.msg import Header

from .modes import (HEADING_MODE_FIN, HEADING_MODE_HEADING, MODE_IGNORE,
                    MODE_UNSET, PITCH_MODE_BATT_POS, PITCH_MODE_PITCH_ONCE,
                    PITCH_MODE_PITCH_SERVO, THRUSTER_MODE_NONE,
                    THRUSTER_MODE_PERCENT_THRUSTER_VOLTAGE,
                    THRUSTER_MODE_POWER)


PITCH_CMD_MAP = {
    MODE_UNSET: UwGliderCommand.PITCH_CMD_NONE,
    MODE_IGNORE: UwGliderCommand.PITCH_CMD_NONE,
    PITCH_MODE_BATT_POS: UwGliderCommand.PITCH_CMD_BATT_POS,
    PITCH_MODE_PITCH_ONCE: UwGliderCommand.PITCH_CMD_TARGET_ONCE,
    PITCH_MODE_PITCH_SERVO: UwGliderCommand.PITCH_CMD_TARGET_SERVO
}

THRUSTER_CMD_MAP = {
    MODE_UNSET: UwGliderCommand.MOTOR_CMD_NONE,
    MODE_IGNORE: UwGliderCommand.MOTOR_CMD_NONE,
    THRUSTER_MODE_NONE: 0,
    THRUSTER_MODE_PERCENT_THRUSTER_VOLTAGE: UwGliderCommand.MOTOR_CMD_VOLTAGE,
    THRUSTER_MODE_POWER: UwGliderCommand.MOTOR_CMD_POWER
}

RUDDER_CMD_MAP = {
    MODE_UNSET: UwGliderCommand.RUDDER_CONTROL_NONE,
    MODE_IGNORE: UwGliderCommand.RUDDER_CONTROL_NONE,
    HEADING_MODE_FIN: UwGliderCommand.RUDDER_CONTROL_ANGLE,
    HEADING_MODE_HEADING: UwGliderCommand.RUDDER_CONTROL_HEADING
}


class RosCmdTopic(object):
    def __init__(self):
        self.sim_command_pub = rospy.Publisher(
            'kinematics/UwGliderCommand',
            UwGliderCommand,
            queue_size=10
        )

    def send_command(self, g):
        msg = UwGliderCommand()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        state = g.state

        # Set pitch!
        msg.pitch_cmd_type = PITCH_CMD_MAP[state.cc_final_pitch_mode]
        if state.cc_final_pitch_mode == PITCH_MODE_BATT_POS:
            # Ooops, glider works in inches, but the message is defined in
            # meters.
            msg.target_pitch_value = state.dc_c_battpos * 2.54/100.0
        elif (state.cc_final_pitch_mode == PITCH_MODE_PITCH_ONCE
              or state.cc_final_pitch_mode == PITCH_MODE_PITCH_SERVO):
            # oops, glider has pitch negative down, but message is negative up.
            msg.target_pitch_value = -state.cc_final_pitch_value

        # Set thruster
        msg.motor_cmd_type = THRUSTER_CMD_MAP[state.cc_final_thruster_mode]
        msg.target_motor_cmd = state.cc_final_thruster_value

        # Set rudder
        msg.rudder_control_mode = RUDDER_CMD_MAP[state.cc_final_heading_mode]
        msg.target_heading = state.cc_final_heading_value
        msg.rudder_angle = UwGliderCommand.RUDDER_ANGLE_DIRECT
        msg.target_rudder_angle = state.cc_final_heading_value

        # Set bouyancy pump
        msg.target_pumped_volume = state.dc_c_oil_volume

        self.sim_command_pub.publish(msg)
