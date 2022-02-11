from six import string_types

from ..behaviors import behavior_class_for_name
from .mission import Mission


def parse_behavior_list(behavior_descs):
    out = []
    for behavior_desc in behavior_descs:
        if isinstance(behavior_desc, string_types):
            name = behavior_desc
            args = {}
        else:
            (name, args), = behavior_desc.items()
        b_class = behavior_class_for_name(name)
        if args is None:
            args = {}
        out.append(b_class(**args))
    return out


class DynamicMission(Mission):
    """A dynamic mission is one where active behaviors can be freely changed at
runtime, typically by some onboard planning and execution process.

    """

    def is_finished(self, g):
        return False

    def add_behavior(self, behavior, g):
        behavior_controls = behavior.CONTROLS
        conflicting_behaviors = [b for b in self.behaviors
                                 if b.CONTROLS.intersection(behavior_controls)]

        for b in conflicting_behaviors:
            b.cancel(g)
            self.behaviors.remove(b)

        self.behaviors.append(behavior)
        behavior.start(g)
        return True

    def stop_behavior(self, behavior, g):
        if behavior in self.behaviors:
            behavior.cancel(g)
            self.behaviors.remove(behavior)
            return True
        return False

    @classmethod
    def from_dict(cls, obj):
        behaviors = parse_behavior_list(obj['initial_behaviors'])
        return cls(behaviors)
