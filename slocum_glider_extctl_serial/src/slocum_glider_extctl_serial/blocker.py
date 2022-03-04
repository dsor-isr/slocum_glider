import logging

import rospy
import serial


serial_logger = logging.getLogger('serial').info


def block_until_start(port):
    ser = serial.Serial(port, baudrate=9600, timeout=1.0)

    while not rospy.is_shutdown():
        line = ser.readline()
        if not line:
            continue
        line = line.strip()
        serial_logger('received: %s', line)
        if line == b'START':
            break

    if not rospy.is_shutdown():
        ser.write(b'Waiting for the $HI\r\n')
        while not rospy.is_shutdown():
            line = ser.readline()
            if not line:
                continue
            line = line.strip()
            if line == b'$HI*01':
                break

    ser.close()
