"""Functionality to retrieve sensor data from simulator over ROS."""

from math import cos, radians

from frl_vehicle_msgs.msg import UwGliderStatus
from uuv_sensor_ros_plugins_msgs.msg import DVL
import rospy
from sensor_msgs.msg import NavSatFix

from .lmc import decimal_degs_to_decimal_mins


class RosSensorsTopic(object):
    def __init__(self):
        self.status_msg = None
        self.dead_reckon_msg = None
        self.altimeter_msg = None
        self.gps_msg = None

        self.sim_status_sub = rospy.Subscriber(
            'glider_hybrid_whoi/kinematics/UwGliderStatus',
            UwGliderStatus,
            self.handle_status_msg
        )
        self.dead_reckoning_sub = rospy.Subscriber(
            'deadreckon',
            NavSatFix,
            self.handle_dead_reckon_msg
        )
        self.gps_sub = rospy.Subscriber(
            'glider_hybrid_whoi/hector_gps',
            NavSatFix,
            self.handle_gps_msg
        )
        self.altimeter_sub = rospy.Subscriber(
            'glider_hybrid_whoi/altimeter',
            DVL,
            self.handle_altimeter_msg
        )

    def handle_status_msg(self, msg):
        self.status_msg = msg

    def handle_dead_reckon_msg(self, msg):
        self.dead_reckon_msg = msg

    def handle_gps_msg(self, msg):
        self.gps_msg = msg

    def handle_altimeter_msg(self, msg):
        self.altimeter_msg = msg

    def update_state(self, g):
        """Given an frl_vehicle_msgs/UwGliderStatus message, update the state instance.

        """
        status_msg = self.status_msg
        self.status_msg = None

        dr_msg = self.dead_reckon_msg
        self.dead_reckon_msg = None

        gps_msg = self.gps_msg
        self.gps_msg = None

        altimeter_msg = self.altimeter_msg
        self.altimeter_msg = None

        state = g.state

        if dr_msg is not None:
            state.m_lat = decimal_degs_to_decimal_mins(dr_msg.latitude)
            state.m_lon = decimal_degs_to_decimal_mins(dr_msg.longitude)

        if gps_msg is not None:
            # The simulator should make sure the message is only published at
            # the surface.
            state.m_gps_lat = decimal_degs_to_decimal_mins(
                gps_msg.latitude
            )
            state.m_gps_lon = decimal_degs_to_decimal_mins(
                gps_msg.longitude
            )
            state.m_gps_status = 0
            state.m_gps_full_status = 0
        else:
            state.m_gps_status = 1
            state.m_gps_full_status = 1

        if status_msg is not None:
            state.m_depth = status_msg.depth
            state.m_roll = status_msg.roll
            state.m_pitch = - status_msg.pitch
            state.m_heading = status_msg.heading

            state.m_thruster_power = status_msg.motor_power

            state.m_fin = status_msg.rudder_angle
            state.m_battpos = status_msg.battery_position
            state.m_de_oil_vol = status_msg.pumped_volume

        if altimeter_msg is not None:
            alt = altimeter_msg.altitude
            # The altimeter is mounted so that it is pointing straight down
            # when the glider is pitched downward by 26 degrees.
            alt = alt * cos(state.m_pitch + radians(26))
            if alt <= state.u_max_altimeter and alt >= state.u_min_altimeter:
                state.m_altitude = alt
                state.m_altimeter_status = 0
            else:
                state.m_altitude = -1
                state.m_altimeter_status = 1
