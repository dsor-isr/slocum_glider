import actionlib
import rospy


class BehaviorActionServer(object):
    """Wraps a behavior in an action server."""

    def __init__(self, name, glider_controller, behavior_class, action_class):
        self.name = name
        self.glider_controller = glider_controller
        self.behavior_class = behavior_class
        self.action_class = action_class

        self.active_goal = None
        self.preempting = False

        self.server = actionlib.SimpleActionServer(name,
                                                   action_class,
                                                   None,
                                                   False)

        self.server.register_goal_callback(self.goal_cb)
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

    def goal_cb(self):
        goal = self.server.accept_new_goal()
        if goal is None:
            return

        rospy.loginfo("Got new goal for behavior %s:\n%s",
                      self.behavior_class,
                      goal)
        if self.server.is_preempt_requested():
            self.server.set_preempted(
                text="Preempted before starting execution"
            )
            return

        behavior = self.behavior_class.from_goal(goal, self)

        result = self.glider_controller.start_behavior(behavior)

        if result:
            self.active_behavior = behavior
        else:
            rospy.logwarn(
                'Unable to activate behavior %s. Has the mission started?',
                self.behavior_class
            )
            self.server.set_aborted(
                text="Unable to activate behavior"
            )

    def preempt_cb(self):
        rospy.loginfo('Goal preempted for %s:\n', self.behavior_class)
        self.preempting = True
        # This could technically return False, but the only way that happens
        # currently is in static missions (where goals will never be accepted)
        # or dynamic missions (in which case this behavior as already ended).
        self.glider_controller.stop_behavior(self.active_behavior)
        rospy.loginfo('Finished preempting for %s:\n', self.behavior_class)
        self.preempting = False

    def send_feedback(self, feedback):
        self.server.publish_feedback(feedback)

    def abort(self, result=None, text=''):
        if self.preempting:
            return
        rospy.loginfo('Aborting goal for %s:\n%s', self.behavior_class, text)
        self.server.set_aborted(result=result, text=text)

    def preempt(self, result=None, text=""):
        if self.preempting:
            return
        rospy.loginfo('Preempting goal for %s:\n%s', self.behavior_class, text)
        self.server.set_preempted(result=result, text=text)

    def succeed(self, result=None, text=""):
        rospy.loginfo('succeeding goal for %s:\n%s', self.behavior_class, text)
        self.server.set_succeeded(result=result, text=text)
