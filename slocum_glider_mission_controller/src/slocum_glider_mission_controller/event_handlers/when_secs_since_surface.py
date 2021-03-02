from .base import EventHandler


class WhenSecsSinceSurfaceHandler(EventHandler):
    """Fires when_secs after the previous surfacing (or start of mission segment).

    """

    EVENT_NAME = 'when_secs_since_surface'

    def __init__(self, behaviors=[], when_secs=60*60):
        super(WhenSecsSinceSurfaceHandler, self).__init__(behaviors)
        self.when_secs = when_secs

    def should_fire(self, g, event):
        return (event['type'] == self.EVENT_NAME
                and event['when_secs'] >= self.when_secs)
