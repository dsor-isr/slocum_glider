"""Functionality to retrieve sensor data from simulator over ROS."""

from frl_vehicle_msgs.msg import UwGliderStatus
import rospy

from .lmc import decimal_degs_to_decimal_mins


class RosSensorsTopic(object):
    def __init__(self):
        self.msg = None
        self.sim_status_sub = rospy.Subscriber('/glider_hybrid_whoi/direct_kinematics/UwGliderStatus', UwGliderStatus,
                                               self.handle_status_msg)

    def handle_status_msg(self, msg):
        self.msg = msg

    def update_state(self, g):
        """Given an frl_vehicle_msgs/UwGliderStatus message, update the state instance.

        """
        msg = self.msg
        self.msg = None
        if msg is None:
            return

        state = g.state

        state.m_lat = decimal_degs_to_decimal_mins(msg.latitude)
        state.m_lon = decimal_degs_to_decimal_mins(msg.longitude)

        # HACK: There is currently no distinction between lat/long coming from
        # gps locks vs dead reackoning. For now, assume GPS if depth is zero,
        # dead reckoning otherwise.
        if msg.depth <= g.state.u_reqd_depth_at_surface:
            state.m_gps_lat = state.m_lat
            state.m_gps_lon = state.m_lon
            state.m_gps_status = 0
            state.m_gps_full_status = 0
        else:
            state.m_gps_status = 1
            state.m_gps_full_status = 1

        state.m_depth = msg.depth
        state.m_roll = msg.roll
        state.m_pitch = msg.pitch
        state.m_heading = msg.heading

        state.m_altitude = msg.altitude

        state.m_thruster_power = msg.motor_power

        state.m_fin = msg.rudder_angle
        state.m_battpos = msg.battery_position
        state.m_de_oil_vol = msg.pumped_volume
