from six import iteritems
from six.moves import collections_abc


class GliderState(collections_abc.MutableMapping):
    """Contains the current values of all sensors."""

    def __init__(self, masterdata):
        sensor_defs = masterdata.sensors
        self.__sensor_defs = sensor_defs
        self.__change_listeners = {}
        self.__update_listeners = {}

        self.__values = {}
        for name, sensor_def in iteritems(sensor_defs):
            self.__values[name.lower()] = sensor_def.default_value

    def add_change_listener(self, name, cb):
        """Register a callback that is fired any time the named sensor changes
        value.

        """
        if name not in self.__change_listeners:
            self.__change_listeners[name] = []

        if cb not in self.__change_listeners[name]:
            self.__change_listeners[name].append(cb)

    def remove_change_listener(self, name, cb):
        if name in self.__change_listeners:
            if cb in self.__change_listeners[name]:
                self.__change_listeners[name].remove(cb)

    def add_update_listener(self, name, cb):
        """Register a callback that is fired any time the named sensor's value is
        set.

        """
        if name not in self.__update_listeners:
            self.__update_listeners[name] = []

        if cb not in self.__update_listeners[name]:
            self.__update_listeners[name].append(cb)

    def remove_update_listener(self, name, cb):
        if name in self.__update_listeners:
            if cb in self.__update_listeners[name]:
                self.__update_listeners[name].remove(cb)

    def __getitem__(self, item):
        return self.__values[item.lower()]

    def __setitem__(self, item, new_value):
        old_value = self[item]
        if new_value is True:
            new_value = 1
        elif new_value is False:
            new_value = 0
        self.__values[item.lower()] = new_value
        if item in self.__update_listeners:
            for cb in self.__update_listeners[item]:
                cb(item, old_value, new_value)
        if item in self.__change_listeners and new_value != old_value:
            for cb in self.__change_listeners[item]:
                cb(item, old_value, new_value)

    def __delitem__(self, item):
        self.__values.__delitem__(item.lower())

    def __iter__(self):
        return self.__values.__iter__()

    def __len__(self):
        return self.__values.__len__()

    def __setattr__(self, name, value):
        if not name.startswith('_'):
            self[name] = value
        else:
            super(GliderState, self).__setattr__(name, value)

    def __getattr__(self, name):
        return self.__values[name.lower()]
