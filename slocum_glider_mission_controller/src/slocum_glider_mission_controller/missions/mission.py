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

    def __init__(self, behaviors, event_handlers=[]):
        self.start_time = rospy.get_time()
        self.last_safe_time = rospy.get_time()
        self.behaviors = behaviors
        self.event_handlers = event_handlers
        self.active_event_handler = None
        self.paused_behaviors = []

    def start(self, g):
        """Start the mission. g is a reference to the glider's extctl interface. Simply
starts every behavior.

        """
        rospy.loginfo('Starting mission with behaviors %s', self.behaviors)
        self.last_time_on_surface = g.state.m_present_time
        enabled_modes = set()
        disabled_modes = set()

        for b in self.behaviors:
            b.start(g)
            modes_to_enable, modes_to_disable = b.desired_modes()
            enabled_modes.update(modes_to_enable)
            disabled_modes.update(modes_to_disable)

        rospy.loginfo(
            'initial mode enable list: %s\ninitial mode disable list: %s',
            enabled_modes,
            disabled_modes
        )
        g.change_modes(list(enabled_modes), list(disabled_modes))
        self.last_enabled_modes = enabled_modes
        self.last_disabled_modes = disabled_modes

    def stop(self, g):
        pass

    def step(self, g):
        """Move the mission along. g is a reference to the glider's extctl
interface. Steps every behavior and removes the ones that have transitioned
into the STOPPED state.

        """
        to_remove = []
        enabled_modes = set()
        disabled_modes = set()

        for b in self.behaviors:
            b.step(g)
            modes_to_enable, modes_to_disable = b.desired_modes()
            enabled_modes.update(modes_to_enable)
            disabled_modes.update(modes_to_disable)
            if b.state == 'STOPPED':
                to_remove.append(b)
        for b in to_remove:
            rospy.loginfo('Removing behavior (state is stopped) %s', b)
            self.behaviors.remove(b)

        if self.last_enabled_modes != enabled_modes \
           or self.last_disabled_modes != disabled_modes:
            rospy.loginfo(
                'Enabling modes %s\nDisabling modes %s',
                enabled_modes,
                disabled_modes
            )
            g.change_modes(list(enabled_modes), list(disabled_modes))
            self.last_enabled_modes = enabled_modes
            self.last_disabled_modes = disabled_modes

        self.fire_default_events(g)

        # If we're handling an event and become unsafe, that means we've
        # finished handling the event.
        if self.active_event_handler and not self.is_actively_controlled():
            self.stop_handling_event(g)

    def fire_default_events(self, g):
        """Fire the default set of events produced by the mission (i.e., not produced
by a specific behavior).

        """

        if (g.state.m_surface_depth_reached and g.state.m_surfacing) \
           or g.state.x_in_surface_dialog > 0:
            self.last_time_on_surface = g.state.m_present_time

        delta_surface = (g.state.m_present_time - self.last_time_on_surface)
        if delta_surface >= 0:
            self.fire_event(g, {'type': 'when_secs_since_surface',
                                'when_secs': delta_surface})

    def fire_event(self, g, event):
        """Determine if any event handler wants to respond to the event. If so, begin
processing it.

        """
        rospy.logdebug('Firing event: %s', event)
        # We currently don't allow nesting event handlers. Once we start
        # responding to one we need to finish it before responding to another.
        if self.active_event_handler:
            return

        for h in self.event_handlers:
            if h.should_fire(g, event):
                self.begin_handling_event(g, h)
                return

    def begin_handling_event(self, g, handler):
        """Register handler as the active event handler. Start all of its behaviors and
add them to the list of active behaviors, pausing any existing behavior that
controls the same things.

        """
        self.active_event_handler = handler
        behaviors = handler.behaviors
        paused_behaviors = []

        # Ensure each behavior is initialized.
        for b in behaviors:
            b.state = 'READY'
            b.start(g)
            # Pause any existing behavior that conflicts.
            for existing_behavior in list(self.behaviors):
                if b.CONTROLS.intersection(existing_behavior.CONTROLS):
                    existing_behavior.pause(g)
                    paused_behaviors.append(existing_behavior)
                    self.behaviors.remove(existing_behavior)

        self.behaviors.extend(behaviors)
        self.paused_behaviors = paused_behaviors
        rospy.loginfo('Entering event handler %s', handler)
        rospy.loginfo('Added %s, paused %s', behaviors, paused_behaviors)

    def stop_handling_event(self, g):
        """Remove the active event handler. Remove all its behaviors from the active
list and resume all paused behaviors.

        """
        handler = self.active_event_handler
        behaviors = handler.behaviors
        paused_behaviors = self.paused_behaviors

        rospy.loginfo('Stoping event handler %s', handler)
        for b in behaviors:
            if b in self.behaviors:
                self.behaviors.remove(b)

        for b in paused_behaviors:
            self.behaviors.append(b)
            b.resume(g)

        self.active_event_handler = None
        self.paused_behaviors = []

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

    def is_actively_controlled(self):
        """Returns True if all required controls are being controlled by some active
behavior.

        """
        actively_controlled = set()
        for b in self.behaviors:
            actively_controlled.update(b.CONTROLS)
        return actively_controlled == self.REQUIRED_CONTROLS

    def is_safe(self):
        """Returns True iff the mission is currently in a safe state."""

        current_time = rospy.get_time()
        if self.is_actively_controlled():
            self.last_safe_time = current_time
            return True
        else:
            diff = current_time - self.last_safe_time
            return diff < self.MAX_UNSAFE_DURATION
