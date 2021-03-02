from abc import ABCMeta, abstractmethod


class EventHandler(object):
    """The base class for events."""
    __metaclass__ = ABCMeta

    def __init__(self, behaviors):
        self.behaviors = behaviors

    @abstractmethod
    def should_fire(self, g, event):
        pass
