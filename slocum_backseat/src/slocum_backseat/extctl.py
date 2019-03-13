import threading

import rospy
import serial
from get_file_service import GetFileService
from send_file_service import SendFileService
from sensors import SensorInterface


def nmea_checksum(s):
    """Return the checksum for the data part of a NMEA sentence as a string."""
    result = 0
    for c in s:
        result ^= ord(c)
    return '%02X' % result


def nmea(s):
    """Add delimiters and checksum to name an NMEA sentence"""
    checksum = nmea_checksum(s)
    return '$' + s + '*' + checksum


def is_valid_nmea_sentence(line):
    """Returns True iff line is a valid NMEA sentence."""
    if len(line) < 4:
        return False
    if line[0] != '$':
        return False
    if line[-3] != '*':
        return False
    got_checksum = line[-2:]
    body = line[1:-3]
    checksum = nmea_checksum(body)
    if checksum.lower() != got_checksum.lower():
        return False
    return True


class SerialInterface:

    def __init__(self, port):
        self.port = port
        self.message_cbs = []
        self.send_lock = threading.Lock()
        self.stop_flag = False

    def add_message_cb(self, cb):
        self.message_cbs.append(cb)

    def remove_message_cb(self, cb):
        self.message_cbs.remove(cb)

    def listener(self):
        while not self.stop_flag:
            line = self.ser.readline()
            if not line:
                continue
            line = line.strip()
            if not is_valid_nmea_sentence(line):
                rospy.logwarn('Rejecting invalid NMEA sentence: %s', line)
                continue
            body = line[1:-3]
            for cb in self.message_cbs:
                cb(body)

    def send_message(self, msg):
        sentence = nmea(msg)
        with self.send_lock:
            self.ser.write(sentence)
            self.ser.write('\r\n')

    def start(self):
        self.stop_flag = False
        self.thread = threading.Thread(target=self.listener)
        self.ser = serial.Serial(self.port, baudrate=9600, timeout=1.0)
        self.thread.start()

    def stop(self):
        self.stop_flag = True
        self.thread.join()
        self.ser.close()


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


class Extctl:
    def __init__(self):
        # Create the serial interface to the glider.
        serial_port_name = rospy.get_param('~serial_port')
        self.ser = SerialInterface(serial_port_name)

        # Start the file getter and sender services. These are independent of
        # information in extctl.ini
        self.file_getter = GetFileService(self.ser)
        self.file_sender = SendFileService(self.ser)

        # We don't necessarily have the extctl.ini file yet, so set
        # self.sensors to None to let the message processor know that.
        self.sensors = None

        # Start processing messages from the glider.
        self.ser.add_message_cb(self.handle_serial_msg)

        # Almost everything is spun up except the sensor interface. Before we
        # can do that we need to make sure we have a valid extctl.ini file.
        sensor_descriptions = self.ensure_extctl_ini()

        # Now instantiate the sensor handler
        writeable = []
        readable = []
        for d in sensor_descriptions:
            if d['writeable']:
                writeable.append((d['name'], d['units']))
            else:
                writeable.append((d['name'], d['units']))
        self.sensors = SensorInterface(writeable, readable, self.ser)

    def ensure_extctl_ini(self):
        """Ensures that a copy of the data from the extctl.ini file is stored on the
        parameter server under the ~extctl/mappings key (see README for
        description).

        If the key is not already set and ~extctl/auto/when_missing is True,
        the file is fetched from the glider with self.fetch_extctl_ini() and
        processed with parse_extctl_ini(). If the key is not set and
        ~extctl/auto/when_missing is False, a ValueError is raised.

        If the key is set, it is used as is.

        Returns the list of dictionaries.

        """

        # Copy of extctl.ini stored under ~extctl/mappings

        if not rospy.has_param('~extctl/mappings'):
            if rospy.get_param('extctl/auto/when_missing'):
                temp = self.fetch_extctl_ini()
                return parse_extctl_ini(temp)
            else:
                raise ValueError()

    def fetch_extctl_ini(self):
        """Fetch the contents of extctl.ini from the glider and return them as a
           string.

        """

        return self.file_getter.get_file('extctl.ini', True)

    def handle_serial_msg(self, msg):
        if msg.startswith('SD,'):
            if self.sensors is None:
                return
            self.sensors.handle_serial_msg(msg)
        elif msg.startswith('FI'):
            self.file_getter.handle_serial_msg(msg)
        else:
            rospy.logwarn('Ignoring unknown sentence: %s', msg)

    def start(self):
        self.ser.start()

    def stop(self):
        self.ser.stop()