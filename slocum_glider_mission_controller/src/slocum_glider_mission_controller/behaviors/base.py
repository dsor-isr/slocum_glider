from abc import ABCMeta, abstractmethod

from ..action_server import BehaviorActionServer


class Behavior(object):
    """The base class for behaviors."""
    __metaclass__ = ABCMeta

    @classmethod
    def make_action_server(cls, backseat_driver):
        # It would be nice to determine the name automatically, but I don't
        # feel like writing a camel case converter and stringcase is not in
        # rosdep database.
        #
        # name = cls.__name__[0:-8].lower()
        return BehaviorActionServer(
            cls.ACTION_NAME,
            backseat_driver,
            cls,
            cls.ACTION
        )

    def __init__(self):
        self.state = 'READY'

    @abstractmethod
    def do_start(self, g):
        pass

    def start(self, g):
        assert self.state == 'READY'
        self.state = 'RUNNING'
        self.do_start(g)

    @abstractmethod
    def do_step(self, g):
        pass

    def step(self, g):
        assert self.state == 'RUNNING'
        self.do_step(g)

    @abstractmethod
    def do_cancel(self, g):
        pass

    def cancel(self, g, text=''):
        assert self.state == 'RUNNING'
        result = self.do_cancel(g)
        if self.server:
            self.server.preempt(result=result, text=text)
        self.state = 'STOPPED'

    @abstractmethod
    def do_stop(self, g):
        pass

    def stop(self, g, text=''):
        assert self.state == 'RUNNING'
        result = self.do_stop(g)
        if self.server:
            self.server.succeed(result=result, text=text)
        self.state = 'STOPPED'
