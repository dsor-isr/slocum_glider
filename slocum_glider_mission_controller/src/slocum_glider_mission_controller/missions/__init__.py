import yaml

from .dynamic_mission import DynamicMission
from .static_mission import StaticMission


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
    return mission_from_dict(obj)
