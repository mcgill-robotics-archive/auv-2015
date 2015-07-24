#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Read from Nucleo serial."""

import rospy
import bitstring
from serial import Serial
from auv_msgs.msg import Signals

__author__ = "Anass Al-Wohoush"

TWELVE_BIT_MODE = False
BUFFERSIZE = 6000

DEBUG_HEADER = "[DEBUG]"
FATAL_HEADER = "[FATAL]"

headers = (
    "[DATA 0]",
    "[DATA 1]",
    "[DATA 2]",
    "[DATA 3]",
    "[DATA 4]",
    DEBUG_HEADER,
    FATAL_HEADER
)

RAW_BUFFERSIZE = 2 * BUFFERSIZE if TWELVE_BIT_MODE else BUFFERSIZE
INT_SIZE = 16 if TWELVE_BIT_MODE else 8


def get_header(ser):
    skipped = 0
    while ser.readable() and not rospy.is_shutdown():
        # Read until next line feed.
        header = ser.readline().strip()

        # Verify if header is valid.
        if header in headers:
            if skipped:
                rospy.logwarn("Skipped %d bytes", skipped)
            return header

        # Otherwise reset.
        skipped += len(header)
        print(header)
        header = ""


def get_data(ser, header):
    # Get data.
    raw = ser.read(RAW_BUFFERSIZE)

    # Determine ADC.
    _, i = header.strip("]").split()
    quadrant = int(i)

    # Convert to array.
    stream = bitstring.BitStream(bytes=raw)
    data = list(
        stream.read(INT_SIZE).uintle
        for i in range(BUFFERSIZE))

    return (quadrant, data)


def iter_data(ser):
    while ser.readable() and not rospy.is_shutdown():
        header = get_header(ser)
        if "DATA" in header:
            yield get_data(ser, header)
        else:
            payload = ser.readline().strip()
            if header == DEBUG_HEADER:
                rospy.logdebug(payload)
            elif header == FATAL_HEADER:
                rospy.logfatal(payload)


if __name__ == "__main__":
    rospy.init_node("nucleo", log_level=rospy.DEBUG)
    pub = rospy.Publisher("~signals", Signals, queue_size=1)
    rospy.loginfo("Assuming %d bit mode", 12 if TWELVE_BIT_MODE else 8)

    with Serial("/dev/nucleo", baudrate=230400) as ser:
        rospy.loginfo("Waiting for device...")
        while not ser.readable():
            pass
        rospy.loginfo("Found device")

        rospy.loginfo("Waiting for data...")
        signals = Signals()
        received = [0 for i in range(4)]
        for quadrant, data in iter_data(ser):
            if quadrant == 1:
                signals.quadrant_1 = data
            elif quadrant == 2:
                signals.quadrant_2 = data
            elif quadrant == 3:
                signals.quadrant_3 = data
            elif quadrant == 4:
                signals.quadrant_4 = data

            received[quadrant] += 1
            if all(q == 1 for q in received):
                pub.publish(signals)
                received = [0 for i in received]
            elif any(q > 1 for q in received):
                rospy.logwarn("Received inconsistent data %r", received)
                received = [0 for i in received]
