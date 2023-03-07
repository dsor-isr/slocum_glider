import itertools
import logging
import threading
import traceback

import rospy
import serial
from six import byte2int, ensure_binary, iterbytes
from slocum_glider_msgs.msg import (Extctl as ExtctlMsg, ExtctlEntry)

from .clear_cached_modes_service import ClearCachedModesService
from .clear_cached_values_service import ClearCachedValuesService
from .get_file_service import GetFileService
from .send_file_service import SendFileService
from .sensors import SensorInterface
from .tty import SerialConsole
from .set_mode_service import SetModeService
from .set_string_service import SetStringService


serial_logger = logging.getLogger('serial').info


def chunked(it, size):
    it = iter(it)
    while True:
        p = tuple(itertools.islice(it, size))
        if not p:
            break
        yield p


def nmea_checksum(s):
    """Return the checksum for the data part of a NMEA sentence as a string."""
    result = 0
    for c in iterbytes(s):
        result ^= c
    return b'%02X' % result


def nmea(s):
    """Add delimiters and checksum to name an NMEA sentence"""
    checksum = nmea_checksum(s)
    return b'$' + s + b'*' + checksum


def is_valid_nmea_sentence(line):
    """Returns True iff line is a valid NMEA sentence."""
    if len(line) < 4:
        return False
    if line[0] != byte2int(b'$'):
        return False
    if line[-3] != byte2int(b'*'):
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
        self.value_lock = threading.Lock()
        self.stop_flag = False

        self.sensor_values = {}
        self.mode_mask = 0
        self.mode_value = 0

    def add_message_cb(self, cb):
        self.message_cbs.append(cb)

    def remove_message_cb(self, cb):
        self.message_cbs.remove(cb)

    def listener(self):
        try:
            self.listener2()
        except Exception as e:
            rospy.logwarn('Listener thread died!: %s', e)
            rospy.logwarn(traceback.format_exc())
            print(e)

    def listener2(self):
        while not self.stop_flag:
            line = self.ser.readline()
            if not line:
                continue
            line = line.strip()
            rospy.logdebug('Received serial message: %s', line)
            serial_logger('received: %s', line)
            is_valid = is_valid_nmea_sentence(line)
            if not is_valid:
                # This is primarily here for startup. During boot, the Glider
                # can put a bunch of crap on the serial line. If the line isn't
                # valid, try finding the latest $ in the line, cutting the
                # beginning of the line, and processing it again.
                last_index = line.rfind(b'$')
                if last_index > 0:
                    line = line[last_index:]
                    is_valid = is_valid_nmea_sentence(line)
            if not is_valid:
                # HACK: the extctl proglet sometimes has trouble transferring
                # files. It can theoretically happen with any size file since
                # it is a timing issue. Therefore, we don't reject any of the
                # file transfer messages for being invalid and just let the
                # file transfer code handle the restarts.
                if line.startswith(b'$FI'):
                    rospy.logwarn('Timing condition in file transfer '
                                  'triggered! Got sentence: %s', line)
                    for cb in self.message_cbs:
                        cb(line)
                else:
                    rospy.logwarn('Rejecting invalid NMEA sentence: %s', line)
                continue
            body = line[1:-3]
            for cb in self.message_cbs:
                cb(body)

    def clear_cached_values(self):
        with self.value_lock:
            self.mode_mask = 0
            self.mode_value = 0
            self.sensor_values = {}

    def clear_cached_modes(self):
        with self.value_lock:
            self.mode_mask = 0
            self.mode_value = 0

    def writer(self):
        try:
            self.writer2()
        except Exception as e:
            rospy.logwarn('Writer thread died!: %s', e)
            rospy.logwarn(traceback.format_exc())
            print(e)

    def writer2(self):
        while not self.stop_flag:
            rospy.sleep(16)
            self.send_all_sensor_values()
            self.send_mode_msg()

    # Assume modes are 0 indexed.
    def activate_modes(self, modes):
        with self.value_lock:
            activate_mask = sum([2**i for i in modes])
            self.mode_mask |= activate_mask
            self.mode_value |= activate_mask

    def deactivate_modes(self, modes):
        with self.value_lock:
            deactivate_mask = sum([2**i for i in modes])
            self.mode_mask |= deactivate_mask
            self.mode_value &= ~deactivate_mask

    def send_mode_msg(self):
        with self.value_lock:
            mask = self.mode_mask
            value = self.mode_value

        if mask != 0:
            rospy.loginfo('Sending mode message mask %s and value %s',
                          mask, value)
            self.send_message(ensure_binary('MD,{},{}'.format(mask, value)))

    def send_sensor_value(self, index, data):
        data = float(data)
        with self.value_lock:
            self.sensor_values[index] = data
        self.send_message(b'SW,%d:%f' % (index, data))
        # rospy.logwarn(b'SW,%d:%f' % (index, data))

    def send_all_sensor_values(self):
        with self.value_lock:
            # Send in chunks of 4. Should moderately more efficient than
            # individually, while keeping us well below the line length.
            for chunk in chunked(self.sensor_values.items(), 4):
                msg = b'SW'
                for index, value in chunk:
                    msg += b',%d:%f' % (index, value)
                self.send_message(msg)

    def send_message(self, msg):
        sentence = nmea(msg)
        with self.send_lock:
            rospy.logdebug('Sending serial message: %s', sentence)
            serial_logger('sending: %s', sentence)
            self.ser.write(sentence)
            self.ser.write(b'\r\n')

    def start(self):
        self.stop_flag = False
        self.read_thread = threading.Thread(target=self.listener)
        self.ser = serial.Serial(self.port, baudrate=9600, timeout=1.0)
        self.write_thread = threading.Thread(target=self.writer)
        self.read_thread.start()
        self.write_thread.start()

    def stop(self):
        self.stop_flag = True
        self.read_thread.join()
        self.write_thread.join()
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
        if(line==''):
          continue
        if(line[0]=='#'):
          continue
        if line[0:2] == 'os' or line[0:2] == 'is' or line[0:2] == 'mp':
            # This is a section header.
            state = line[0:2]
        elif state:
            try:
              name, units = line.split()
            except ValueError:	# Assume the error is due to only one parameter (as is the case for 'mp')
              name = line
              units = 'nodim'

            out.append({'name': name,
                        'units': units,
                        'writeable': state == 'os' or state == 'mp'})
        else:
            raise ValueError('Invalid extctl.ini file')
    return out


class Extctl:
    def __init__(self):
        # Keep track of if extctl is sending us stuff. Starts True because the
        # blocker waits for a $HI message.
        self.active = True

        # Create the serial interface to the glider.
        serial_port_name = rospy.get_param('~serial_port/device')
        self.serial_port_name = serial_port_name
        self.ser = SerialInterface(serial_port_name)

        # Start the file getter and sender services. These are independent of
        # information in extctl.ini
        self.file_getter = GetFileService(self.ser)
        self.file_sender = SendFileService(self.ser)

        # Start the sciense mission mode service
        self.mode_setter = SetModeService(self.ser)

        # Start the string setting service for debugging purposes
        self.string_setter = SetStringService(self.ser)

        # Start the services to clear cached values.
        self.clear_cached_values_service = ClearCachedValuesService(self.ser)
        self.clear_cached_modes_service = ClearCachedModesService(self.ser)

        # The publisher for the extctl.ini file.
        self.extctl_pub = rospy.Publisher('extctl/ini', ExtctlMsg,
                                          queue_size=1, latch=True)

        # We don't necessarily have the extctl.ini file yet, so set
        # self.sensors to None to let the message processor know that.
        self.sensors = None

        # Start processing messages from the glider.
        self.ser.add_message_cb(self.handle_serial_msg)

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
            if rospy.get_param('~extctl/auto/when_missing'):
                temp = self.fetch_extctl_ini()
                return parse_extctl_ini(temp)
            else:
                raise ValueError()

    def fetch_extctl_ini(self):
        """Fetch the contents of extctl.ini from the glider and return them as a
           string.

        """

        return self.file_getter.get_file('extctl.ini', True).decode('utf-8')

    def handle_serial_msg(self, msg):
        if msg.startswith(b'SD,'):
            if self.sensors is None:
                return
            self.sensors.handle_serial_msg(msg)
        elif msg.startswith(b'FI') or msg.startswith(b'$FI'):
            self.file_getter.handle_serial_msg(msg)
        elif msg == b'TT':
            self.ser.send_message(b'TS,S')
            serial_console = SerialConsole(self.serial_port_name)
            self.ser.ser.close()
            serial_console.run()
            self.ser.ser = serial.Serial(self.serial_port_name, baudrate=9600,
                                         timeout=1.0)
        elif msg == b'HI':
            self.active = True
            rospy.loginfo('extctl sent a $HI')
        elif msg == b'BY':
            self.active = False
            rospy.loginfo('extctl sent a $BY')
        else:
            rospy.logwarn('Ignoring unknown sentence: %s', msg)

    def start(self):
        self.ser.start()
        # Almost everything is spun up except the sensor interface. Before we
        # can do that we need to make sure we have a valid extctl.ini file.
        sensor_descriptions = self.ensure_extctl_ini()

        msg = ExtctlMsg()
        # Now instantiate the sensor handler
        writeable = []
        readable = []
        for d in sensor_descriptions:
            if d['writeable']:
                msg.backseat_outputs.append(ExtctlEntry(d['name'], d['units']))
                writeable.append((d['name'], d['units']))
            else:
                msg.backseat_inputs.append(ExtctlEntry(d['name'], d['units']))
                readable.append((d['name'], d['units']))
        self.extctl_pub.publish(msg)
        self.sensors = SensorInterface(writeable, readable, self.ser)

    def stop(self):
        self.ser.stop()
