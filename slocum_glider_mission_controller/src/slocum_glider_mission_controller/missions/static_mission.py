from copy import deepcopy
from six import string_types

from .mission import Mission
from ..behaviors import behavior_class_for_name
from ..event_handlers import event_handler_class_for_name


class StaticMissionSegment(object):
    def __init__(self, behaviors, event_handlers):
        self.behaviors = behaviors
        self.event_handlers = event_handlers


def parse_behavior_list(behavior_descs):
    out = []
    for behavior_desc in behavior_descs:
        (name, args), = behavior_desc.items()
        b_class = behavior_class_for_name(name)
        if args is None:
            args = {}
        out.append(b_class(**args))
    return out


class StaticMission(Mission):
    """A static mission disallows runtime editing of the list of behaviors.

A static mission is finished if any of the behaviors transition into a stopped
state.

    """

    def __init__(self, segments):
        super(StaticMission, self).__init__(list(segments[0].behaviors),
                                            list(segments[0].event_handlers))
        self.segments = segments

    def is_finished(self, g):
        return self.segments == []

    def add_behavior(self, behavior, g):
        return False

    def stop_behavior(self, behavior, g):
        return False

    def step(self, g):
        super(StaticMission, self).step(g)
        if not self.is_actively_controlled():
            self.segments.pop(0)
            if self.segments:
                self.behaviors = list(self.segments[0].behaviors)
                self.event_handlers = list(self.segments[0].event_handlers)
                self.paused_behaviors = []
                self.active_event_handler = None
                for b in self.behaviors:
                    b.start(g)

    @classmethod
    def from_dict(cls, obj):
        segments = []
        default_handler_descs = obj.get('event_handlers', [])
        for segment_desc in obj['segments']:
            behaviors = parse_behavior_list(segment_desc['behaviors'])
            event_handlers = []
            for event_handler_desc in (segment_desc.get('event_handlers', [])
                                       + deepcopy(default_handler_descs)):
                (name, args), = event_handler_desc.items()
                handler_class = event_handler_class_for_name(name)
                args['behaviors'] = parse_behavior_list(args['behaviors'])
                event_handlers.append(handler_class(**args))
            segments.append(StaticMissionSegment(behaviors, event_handlers))
        return cls(segments)
