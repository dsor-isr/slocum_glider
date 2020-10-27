"""Implementation of the extctl proglet."""

from contextlib import closing

from six import iteritems

import rospy
from std_msgs.msg import Byte, Float64, Float32
from slocum_glider_msgs.srv import (SetByte, SetFloat32, SetFloat64, SetMode,
                                    SetModeResponse)

# Translation from glider "units" to ROS message types.
GLIDER_MSG_TYPES = {
    'bool':      Float32,
    'byte':      Byte,
    'enum':      Byte,
    'lat':       Float64,
    'lon':       Float64,
    'timestamp': Float64,
    'X':         Float64
    }


GLIDER_SRV_TYPES = {
    'bool':      SetFloat32,
    'byte':      SetByte,
    'enum':      SetByte,
    'lat':       SetFloat64,
    'lon':       SetFloat64,
    'timestamp': SetFloat64,
    'X':         SetFloat64
}


def get_msg_type(units):
    """Convert a glider unit to a ROS message type."""
    if units in GLIDER_MSG_TYPES:
        return GLIDER_MSG_TYPES[units]
    return Float32


def get_srv_type(units):
    """Convert a glider unit to a ROS service type."""
    if units in GLIDER_SRV_TYPES:
        return GLIDER_SRV_TYPES[units]
    return SetFloat32


def parse_extctl_ini(s):
    """Given a string of the contents of the extctl.ini file, return a list of
    dictionaries describing the file, suitable for storage in the ROS parameter
    server.

    Each dictionary has the following keys: writeable (boolean), name (string),
    and units (string).

    """
    state = None
    out = []
    for rline in s.splitlines():
        line = rline.strip()
        if line == 'os' or line == 'is':
            # This is a section header.
            state = line
        elif state:
            name, units = line.split()
            out.append({'name': name,
                        'units': units,
                        'writeable': state == 'os'})
        else:
            raise ValueError('Invalid extctl.ini file')
    return out


class ExtctlProglet(object):
    def __init__(self, g):
        self.g = g
        science_fs = g.science_fs
        if science_fs.exists(['config', 'extctl.ini']):
            with closing(g.open_science_file(['config', 'extctl.ini'])) as f:
                contents = f.read()
        else:
            contents = "os\nis\n"

        self.extctl_ini = parse_extctl_ini(contents)
        self.init_topics_and_srvs()

    def init_topics_and_srvs(self):
        pubs = {}
        srvs = {}
        self.pubs = pubs
        self.srvs = srvs
        for sensor in self.extctl_ini:
            name = sensor['name']
            if not sensor['writeable']:
                pubs[name] = rospy.Publisher('extctl/sensors/' + name,
                                             get_msg_type(sensor['units']),
                                             queue_size=1)
            else:
                srv_type = get_srv_type(sensor['units'])
                srvs[name] = rospy.Service(
                    'extctl/sensors/set_' + name,
                    srv_type,
                    self.make_srv_handler(sensor, srv_type._response_class))

        self.mode_srv = rospy.Service('extctl/set_mode',
                                      SetMode,
                                      self.set_mode)

    def make_srv_handler(self, sensor, response_type):
        name = sensor['name']

        def handler(req):
            data = req.data
            self.g.state[name] = data
            return response_type(success=True)

        return handler

    def send_sensor_values(self):
        for (name, pub) in iteritems(self.pubs):
            pub.publish(self.g.state[name])

    def set_mode(self, req):
        modes_to_activate = req.modes_to_activate
        modes_to_deactivate = req.modes_to_deactivate

        # Check that the same mode is not being both activated and deactivated
        coll = set(modes_to_activate) & set(modes_to_deactivate)
        if coll:
            return SetModeResponse(
                success=False,
                message="Cannot activate & deactivate mode(s): {}".format(list(coll))  # NOQA: E501
            )

        # Compute mask to activate (assume modes are 1-indexed)
        if modes_to_activate:
            activate_mask = sum([2**(i-1) for i in modes_to_activate])
        else:
            activate_mask = 0

        if modes_to_deactivate:
            deactivate_mask = sum([2**(i-1) for i in modes_to_activate])
        else:
            deactivate_mask = 0

        old_sci_mission_mode = self.g.state.sci_mission_mode

        old_sci_mission_mode |= activate_mask
        old_sci_mission_mode &= ~deactivate_mask
        self.g.state.sci_mission_mode = old_sci_mission_mode
