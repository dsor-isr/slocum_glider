from abc import ABCMeta, abstractmethod

from ..action_server import BehaviorActionServer


class Behavior(object):
    """The base class for behaviors."""
    __metaclass__ = ABCMeta
    DEFAULT_MODES_DISABLED = []
    DEFAULT_MODES_ENABLED = []

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
        self._modes_enabled = list(self.DEFAULT_MODES_ENABLED)
        self._modes_disabled = list(self.DEFAULT_MODES_DISABLED)

    def desired_modes(self):
        return self._modes_enabled, self._modes_disabled

    def enable_mode(self, mode_to_enable):
        if mode_to_enable not in self._modes_enabled:
            self._modes_enabled.append(mode_to_enable)
        if mode_to_enable in self._modes_disabled:
            self._modes_disabled.remove(mode_to_enable)

    def disable_mode(self, mode_to_disable):
        if mode_to_disable not in self._modes_disabled:
            self._modes_disabled.append(mode_to_disable)
        if mode_to_disable in self._modes_enabled:
            self._modes_enabled.remove(mode_to_disable)

    @abstractmethod
    def do_start(self, g):
        pass

    def start(self, g):
        assert self.state == 'READY'
        self.state = 'RUNNING'
        self.do_start(g)

    def do_pause(self, g):
        pass

    def pause(self, g):
        assert self.state == 'RUNNING'
        self.state = 'PAUSED'
        self.do_pause(g)

    def do_resume(self, g):
        pass

    def resume(self, g):
        assert self.state == 'PAUSED'
        self.state = 'RUNNING'
        self.do_resume(g)

    @abstractmethod
    def do_step(self, g):
        pass

    def step(self, g):
        assert self.state == 'RUNNING'
        self.do_step(g)

    @abstractmethod
    def do_abort(self, g):
        pass

    def abort(self, g, text=''):
        assert self.state == 'RUNNING'
        result = self.do_abort(g)
        if self.server:
            self.server.abort(result=result, text=text)
        self.state = 'STOPPED'

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
