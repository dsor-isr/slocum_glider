import rospy
from std_msgs.msg import Bool, Byte, Float64, Float32
from slocum_glider_msgs.srv import SetBool, SetByte, SetFloat32, SetFloat64

# Translation from glider "units" to ROS message types.
GLIDER_MSG_TYPES = {
    'bool':      Bool,
    'byte':      Byte,
    'enum':      Byte,
    'lat':       Float64,
    'lon':       Float64,
    'timestamp': Float64,
    'X':         Float64
    }


GLIDER_SRV_TYPES = {
    'bool':      SetBool,
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


class ReadableSensor:
    """Represents a sensor that can be read from the glider."""
    def __init__(self, name, units):
        self.name = name
        self.units = units
        self.ros_type = get_msg_type(units)
        self.publisher = rospy.Publisher('extctl/sensors/' + name,
                                         self.ros_type,
                                         queue_size=10,
                                         latch=True)

    def publish(self, value):
        self.publisher.publish(value)


class WriteableSensor:
    """Represents a sensor that can be written on the glider."""
    def __init__(self, name, index, units, ser):
        self.name = name
        self.units = units
        self.ros_type = get_srv_type(units)
        self.response_type = self.ros_type._response_class
        self.ser = ser
        self.index = index
        self.service = rospy.Service('extctl/sensors/set_' + name,
                                     self.ros_type,
                                     self.handler)

    def handler(self, req):
        """Send a message to the glider setting the new value.

        All type information is discarded.

        """
        data = req.data
        self.ser.send_message('SW,%d:%g' % (self.index, float(data)))
        return self.response_type(success=True)


class SensorInterface:
    def __init__(self, writeable, readable, ser):
        index = 0
        self.sensors = []
        for name, units in writeable:
            self.sensors.append(WriteableSensor(name, index, units, ser))
            index += 1

        for name, units in readable:
            self.sensors.append(ReadableSensor(name, units))
            index += 1

    def handle_serial_msg(self, data):
        fields = data.split(',')
        if fields[0] != 'SD':
            raise ValueError('Invalid data')

        del fields[0]

        for s in fields:
            (index_str, val_str) = s.split(':')
            value = float(val_str)
            index = int(index_str)
            self.sensors[index].publish(value)
