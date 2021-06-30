from .behavior import Behavior
from ..modes import MODE_IGNORE


class NopCmdsBehavior(Behavior):
    """Behavior to keep the stack busy."""
    NAME = 'nop_cmds'
    HAS_WAITING_STATE = False

    def compute_controls(self, x):
        if self.args.nop_bpump:
            x.cc_bpump_mode = MODE_IGNORE
        if self.args.nop_heading:
            x.cc_heading_mode = MODE_IGNORE
        if self.args.nop_pitch:
            x.cc_pitch_mode = MODE_IGNORE
        if self.args.nop_threng:
            x.cc_threng_mode = MODE_IGNORE

    def should_stop(self, x):
        return False
