import rospy

from sensors import SensorInterface
from get_file_service import GetFileService

import serial
import threading


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
    state = None
    writeable = []
    readable = []
    for rline in s.splitlines():
        line = rline.strip()
        # section header
        if line == 'os' or line == 'is':
            state = line
            continue
        elif state:
            (name, units) = line.split()
            if state == 'os':
                writeable.append((name, units))
            else:
                readable.append((name, units))
        else:
            raise ValueError('Invalid extctl.ini file')
    return writeable, readable


class Extctl:
    def __init__(self):
        serial_port_name = rospy.get_param('~serial_port')

        self.ser = SerialInterface(serial_port_name)

        ini_string = rospy.get_param('~extctl_ini')

        writeable, readable = parse_extctl_ini(ini_string)
        self.sensors = SensorInterface(writeable, readable, self.ser)

        self.file_getter = GetFileService(self.ser)

        self.ser.add_message_cb(self.handle_serial_msg)

    def handle_serial_msg(self, msg):
        if msg.startswith('SD,'):
            self.sensors.handle_serial_msg(msg)
        elif msg.startswith('FI'):
            self.file_getter.handle_serial_msg(msg)
        else:
            rospy.logwarn('Ignoring unknown sentence: %s', msg)

    def start(self):
        self.ser.start()

    def stop(self):
        self.ser.stop()
