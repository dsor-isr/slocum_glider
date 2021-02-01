"""Easy access to the glider's state via the extctl protocol over ROS"""


from threading import Lock, Condition

import rospy
from six import iteritems
from six.moves import collections_abc

from slocum_glider_msgs.msg import Extctl
from slocum_glider_msgs.srv import (GetFile, SetByte, SetFloat32, SetFloat64,
                                    SetMode)
from std_msgs.msg import Byte, Float64, Float32

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


class GliderState(collections_abc.MutableMapping):
    """Expose the glider state as on object and dictionary. Sensor values can be
read or written as attributes or items.

Writes will be proxied to the appropriate extctl service. Reads are cached
from extctl topics.

    """
    def __init__(self, parent, values={}):
        super(GliderState, self).__setattr__('_parent', parent)
        super(GliderState, self).__setattr__('_values', values)

    def __delitem__(self, item):
        pass

    def __getitem__(self, item):
        return self._values[item]

    def __setitem__(self, item, new_value):
        self._parent.set_sensor(item, new_value)

    def __iter__(self):
        return self._values.__iter__()

    def __len__(self):
        return self._values.__len__()

    def __setattr__(self, name, value):
        self[name] = value

    def __getattr__(self, name):
        return self._values[name]


class GliderExtctlInterface(object):
    """A Pythonic interface to the ROS extctl interface.

    """

    def __init__(self):
        self._extctl_sub = rospy.Subscriber('extctl/ini', Extctl,
                                            self._extctl_cb)
        self._values = {}
        self._backseat_inputs = {}
        self._backseat_outputs = {}
        self._lock = Lock()
        self._have_all_inputs = False
        self._have_all_inputs_condition = Condition(self._lock)
        self._mode_srv = rospy.ServiceProxy('extctl/set_mode', SetMode)
        self._get_file_srv = rospy.ServiceProxy('extctl/get_file', GetFile)
        self.state = GliderState(self, self._values)

    def _extctl_cb(self, msg):
        _backseat_inputs = {}
        _backseat_outputs = {}
        for entry in msg._backseat_outputs:
            _backseat_outputs[entry.name] = get_srv_type(entry.units)
        for entry in msg._backseat_inputs:
            _backseat_inputs[entry.name] = get_msg_type(entry.units)

        with self._lock:
            self._register_topics(_backseat_inputs)
            self._register_srvs(_backseat_outputs)

    def _make_topic_cb(self, name):
        def cb(msg):
            with self._lock:
                self._values[name] = msg.data
                if not self._have_all_inputs:
                    keys_received = set(self._values.keys())
                    all_keys = set(self._backseat_inputs.keys())
                    if keys_received == all_keys:
                        self._have_all_inputs = True
                        self._have_all_inputs_condition.notify_all()
        return cb

    def _register_topics(self, _backseat_inputs):
        for (name, sub) in iteritems(self._backseat_inputs):
            if name not in _backseat_inputs:
                sub.unregister()
                del self.backseat_intputs[name]
                if name in self._values:
                    del self._values[name]
        for (name, msg_type) in iteritems(_backseat_inputs):
            if name not in self._backseat_inputs:
                self._backseat_inputs[name] = rospy.Subscriber(
                    'extctl/sensors/' + name,
                    msg_type,
                    self._make_topic_cb(name)
                )
                self._have_all_inputs = False

    def _register_srvs(self, _backseat_outputs):
        for (name, proxy) in iteritems(self._backseat_outputs):
            if name not in _backseat_outputs:
                del self._backseat_outputs[name]
        for (name, srv_type) in iteritems(_backseat_outputs):
            if name not in self._backseat_outputs:
                self._backseat_outputs[name] = rospy.ServiceProxy(
                    'extctl/sensors/set_' + name,
                    srv_type
                )

    def change_modes(self, enable, disable):
        self._mode_srv(enable, disable)

    def get_file(self, name, block=True):
        res = self._get_file_srv(GetFile(name=name, block=block))
        return res.success, res.contents

    def set_sensor(self, name, value):
        self._backseat_outputs[name](value)

    def snapshot(self):
        with self._lock:
            return GliderExtctlProxy(
                self,
                GliderState(self, self._values.copy())
            )

    def wait_for_all_inputs(self, timeout=None):
        with self._lock:
            if self._have_all_inputs:
                return True
            self._have_all_inputs_condition.wait(timeout=timeout)
            return self._have_all_inputs


class GliderExtctlProxy(object):
    """Caches the state locally. Proxies remaining operations to the parent.

    """
    def __init__(self, parent, state):
        self.state = state
        self._parent = parent

    def __getattr__(self, name):
        return getattr(self._parent, name)
