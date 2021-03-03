import yaml

from .dynamic_mission import DynamicMission
from .static_mission import StaticMission


class NoAliasDumper(yaml.Dumper):
    def ignore_aliases(self, data):
        return True


def mission_from_dict(mission_dict):
    (mission_type, args), = mission_dict.items()
    if mission_type == 'dynamic_mission':
        return DynamicMission.from_dict(args)
    if mission_type == 'static_mission':
        return StaticMission.from_dict(args)
    else:
        raise KeyError('Unknown mission type ' + mission_type)


def mission_from_yaml_string(yaml_string):
    obj = yaml.safe_load(yaml_string)
    # Redump the object to yaml, without anchors, and reparse it. This is
    # annoying but is the best way to make sure there's no shared structure
    # within the object (introduced by anchors). If this is too much of a
    # performance hit (very unlikely), we can take greater pains to not reuse
    # objects from this object elsewhere in the code.
    obj = yaml.safe_load(yaml.dump(obj, Dumper=NoAliasDumper))
    return mission_from_dict(obj)
