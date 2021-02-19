from .mission import Mission
from ..behaviors import behavior_class_for_name


class StaticMissionSegment(object):
    def __init__(self, behaviors):
        self.behaviors = behaviors


class StaticMission(Mission):
    """A static mission disallows runtime editing of the list of behaviors.

A static mission is finished if any of the behaviors transition into a stopped
state.

    """

    def __init__(self, segments):
        super(StaticMission, self).__init__(list(segments[0].behaviors))
        self.segments = segments

    def is_finished(self, g):
        return self.segments == []

    def add_behavior(self, behavior, g):
        return False

    def stop_behavior(self, behavior, g):
        return False

    def step(self, g):
        super(StaticMission, self).step(g)
        if self.behaviors != self.segments[0].behaviors:
            self.segments.pop(0)
            if self.segments:
                self.behaviors = list(self.segments[0].behaviors)
                for b in self.behaviors:
                    b.start(g)

    @classmethod
    def from_dict(cls, obj):
        segments = []
        for segment_desc in obj['segments']:
            behaviors = []
            for behavior_desc in segment_desc['behaviors']:
                (name, args), = behavior_desc.items()
                b_class = behavior_class_for_name(name)
                if args is None:
                    args = {}
                behaviors.append(b_class(**args))
            segments.append(StaticMissionSegment(behaviors))
        return cls(segments)
