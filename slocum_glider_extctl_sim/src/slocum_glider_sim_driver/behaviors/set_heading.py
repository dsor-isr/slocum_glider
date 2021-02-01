from .behavior import Behavior, maybe_deref


class SetHeadingBehavior(Behavior):
    NAME = 'set_heading'

    def compute_controls(self, x):

        x.cc_heading_mode = self.args.use_heading
        x.cc_heading_value = maybe_deref(self.args.heading_value, x)
