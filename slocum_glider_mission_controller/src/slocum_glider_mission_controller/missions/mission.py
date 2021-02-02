from abc import ABCMeta, abstractmethod

import rospy


class Mission(object):
    """A mission object.

A mission consists of a list of behaviors that are active. Behaviors can be
added or removed depending on the policies of the concrete implementations.

    """
    __metaclass__ = ABCMeta

    REQUIRED_CONTROLS = set(['heading', 'pitch', 'bpump', 'thruster'])
    MAX_UNSAFE_DURATION = 8

    def __init__(self, behaviors):
        self.start_time = rospy.get_time()
        self.last_safe_time = rospy.get_time()
        self.behaviors = behaviors

    def start(self, g):
        """Start the mission. g is a reference to the glider's extctl interface. Simply
starts every behavior.

        """
        for b in self.behaviors:
            b.start(g)

    def step(self, g):
        """Move the mission along. g is a reference to the glider's extctl
interface. Steps every behavior and removes the ones that have transitioned
into the STOPPED state.

        """
        to_remove = []
        for b in self.behaviors:
            b.step(g)
            if b.state == 'STOPPED':
                to_remove.append(b)
        for b in to_remove:
            self.behaviors.remove(b)

    def end(self, g):
        for b in self.behaviors:
            b.cancel(g)

    @abstractmethod
    def is_finished(self, g):
        """Returns True iff the mission is finished."""
        pass

    @abstractmethod
    def add_behavior(self, behavior, g):
        """Add a behavior to the set of active behaviors. Returns True iff
successful.

        """
        pass

    @abstractmethod
    def stop_behavior(self, behavior, g):
        """Remove a behavior from the set of active behaviors. Returns True iff
successful.

        """
        pass

    def is_safe(self):
        """Returns True iff the mission is currently in a safe state."""
        actively_controlled = set()
        for b in self.behaviors:
            actively_controlled.update(b.CONTROLS)

        current_time = rospy.get_time()
        if actively_controlled == self.REQUIRED_CONTROLS:
            self.last_safe_time = current_time
            return True
        else:
            diff = current_time - self.last_safe_time
            return diff > self.MAX_UNSAFE_DURATION
