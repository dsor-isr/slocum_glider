from slocum_glider_msgs.msg import SurfaceAction, SurfaceResult

from .base import Behavior
from ..modes import MODE_NORMAL_SURFACE_BIT


class SurfaceBehavior(Behavior):
    """A behavior that asks the glider to surface.

    """

    ACTION = SurfaceAction
    ACTION_NAME = 'surface'
    CONTROLS = set(['pitch', 'bpump'])
    DEFAULT_MODES_ENABLED = [MODE_NORMAL_SURFACE_BIT]

    def __init__(self, server=None):
        super(SurfaceBehavior, self).__init__()
        self.server = server

    @classmethod
    def from_goal(cls, goal, server):
        return cls(server=server)

    def do_start(self, g):
        self.previous_in_surface_dialog = g.state.x_in_surface_dialog

    def do_step(self, g):
        if self.previous_in_surface_dialog and not g.state.x_in_surface_dialog:
            self.stop(g)
        else:
            self.previous_in_surface_dialog = g.state.x_in_surface_dialog

        if g.state.x_in_surface_dialog:
            # We need to clear this mode otherwise we'll never stop trying to
            # surface!
            g.change_modes([], [MODE_NORMAL_SURFACE_BIT])

    def do_abort(self, g):
        pass

    def do_cancel(self, g):
        pass

    def do_stop(self, g):
        return SurfaceResult()
