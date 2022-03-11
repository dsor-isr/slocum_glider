"""The actual backseat driver"""

from threading import Lock
import traceback

import rospy
from slocum_glider_extctl import GliderExtctlInterface

from .behaviors import BEHAVIOR_CLASSES
from .missions import mission_from_yaml_string
from .modes import MODE_EMERGENCY_SURFACE_BIT, MODE_NORMAL_SURFACE_BIT


class GliderController(object):
    """Implements the actionlib controller for the Glider.

spin is a blocking function that implements a 0.5 Hz control loop (the glider's
control loop is 0.25 Hz).

Whenever the glider starts a mission (determined by watching x_in_gliderdos), a
new Mission object is created. This mission is steped along every time through
the control loop while the glider still has a mission ongoing. If the glider
ends its mission, the backseat driver mission is also ended.

If the BSD's mission ever becomes unsafe (critical pieces of the glider aren't
being actively controlled, etc.), the BSD mission is immediately ended and the
glider is emergency surfaced.

    """

    def __init__(self):
        self.extctl = GliderExtctlInterface()
        self.mission = None
        self.mission_lock = Lock()
        action_servers = []
        for b in BEHAVIOR_CLASSES:
            action_servers.append(b.make_action_server(self))
        self.action_servers = action_servers

    def start_behavior(self, behavior):
        with self.mission_lock:
            g = self.extctl.snapshot()
            if self.mission:
                return self.mission.add_behavior(behavior, g)
            return False

    def stop_behavior(self, behavior):
        with self.mission_lock:
            g = self.extctl.snapshot()
            if self.mission:
                return self.mission.stop_behavior(behavior, g)
            return False

    def out_of_band_abort(self):
        """An out of band abort is triggered if the active mission ever finishes or
enters an unsafe state. It (currently) simply requests that the glider perform
an emergency surface.

Future iterations will likely also sprial in place and set the thruster to max.

        """
        rospy.logerr('OUT OF BAND ABORT!')
        self.extctl.change_modes([MODE_EMERGENCY_SURFACE_BIT], [])

    def spin(self):
        # We are not ready yet!
        self.extctl.wait_for_extctl()
        self.extctl.state.u_mission_param_k = 0

        # Wait for us to have a complete view of the glider sensors.
        rospy.loginfo('Waiting for all inputs')
        self.extctl.wait_for_all_inputs()
        rospy.loginfo('All inputs received')

        gave_status = False

        rate = rospy.Rate(0.5)
        try:
            while not rospy.is_shutdown():
                rate.sleep()
                with self.mission_lock:
                    g = self.extctl.snapshot()
                    # Tell the user that we're ready!
                    g.state.u_mission_param_k = 1
                    # Clear the cached modes from the serial driver if the user
                    # has also asked the glider to clear them.
                    if g.state.u_mission_mode_clear \
                       or g.state.u_mission_param_o:
                        g.change_modes([], [i for i in range(8)])
                        g.clear_cached_modes()

                    if (not g.state.x_in_gliderdos) \
                       and g.state.u_mission_param_l == 1:
                        # We are in a mission that we can control. Start a
                        # mission if none are currently active.
                        if self.mission is None:
                            rospy.loginfo('Beginning mission')
                            n = g.state.u_mission_param_n
                            _, mission_str = g.get_file('backse%02d.ini' % n)
                            rospy.loginfo('mission: %s', mission_str)
                            self.mission = mission_from_yaml_string(
                                mission_str
                            )
                            self.mission.start(g)
                        mission = self.mission
                        mission.step(g)
                        is_finished = mission.is_finished(g)
                        is_safe = mission.is_safe()
                        if (is_finished or not is_safe):
                            if is_finished:
                                rospy.loginfo('Mission finished')
                            else:
                                rospy.logerr('Mission is unsafe')
                            self.mission = None
                            self.out_of_band_abort()
                    elif ((not g.state.x_in_gliderdos)
                          and g.state.u_mission_param_l == 2):
                        # user has asked for a status update!
                        if gave_status:
                            rospy.logdebug(
                                'Already provided status, continuing'
                            )
                            continue
                        gave_status = True
                        rospy.loginfo('Sending bsd.log')
                        g.send_file(
                            'bsd.log',
                            'The backseat driver is operating normally.\n'
                        )
                        self.extctl.change_modes([MODE_NORMAL_SURFACE_BIT,
                                                  MODE_EMERGENCY_SURFACE_BIT],
                                                 [])
                    else:
                        # Not in a mission we can control. End the current
                        # mission if one is set.
                        gave_status = False
                        if self.mission:
                            rospy.loginfo(
                                'Glider mission finished, ending my mission'
                            )
                            self.mission.end(g)
                            self.mission = None
        except Exception as ex:
            print('Unhandled exception', ex)
            print(traceback.format_exc())
            print('Aborting!')
            rospy.logerr('Unhandled exception %s', ex)
            rospy.logerr('Traceback: %s', traceback.format_exc())
            self.out_of_band_abort()
