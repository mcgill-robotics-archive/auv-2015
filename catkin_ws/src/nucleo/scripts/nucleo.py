#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Nucleo deserializer."""

import rospy
import bitstring
from serial import Serial
from datetime import datetime
from auv_msgs.msg import Signals

__author__ = "Anass Al-Wohoush"

# Define bootup header.
BOOTUP_HEADER = "[BOOTUP]"

# Define logging headers.
DEBUG_HEADER = "[DEBUG]"
FATAL_HEADER = "[FATAL]"

# Define all available headers.
headers = (
    "[DATA 1]",
    "[DATA 2]",
    "[DATA 3]",
    "[DATA 4]",
    DEBUG_HEADER,
    FATAL_HEADER,
    BOOTUP_HEADER
)

# Preserve messed up and skipped headers.
corrupted_log = open("{}_corrupted.log".format(datetime.now()), "a+")


def get_header(ser):
    """Gets next header from serial buffer.

    Args:
        ser: Serial port.

    Returns:
        Header.
    """
    while ser.readable() and not rospy.is_shutdown():
        # Read until next line feed.
        header = ser.readline()

        # Verify if header is valid.
        if header.strip() in headers:
            return header

        # Otherwise reset.
        if header and header != "\n":
            rospy.logerr("Skipped %d bytes", len(header))
            print >> corrupted_log, header,
        header = ""


def get_data(ser, header):
    """Gets next payload.

    Args:
        ser: Serial port.
        header: Corresponding header.

    Returns:
        Tuple of (quadrant, data).
    """
    # Get data.
    raw = ser.read(RAW_BUFFERSIZE)

    # Determine quadrant.
    if "DATA 1" in header:
        quadrant = 1
    elif "DATA 2" in header:
        quadrant = 2
    elif "DATA 3" in header:
        quadrant = 3
    elif "DATA 4" in header:
        quadrant = 4

    # Convert to array.
    stream = bitstring.BitStream(bytes=raw)
    data = list(
        stream.read(INT_SIZE).uintle
        for i in range(BUFFERSIZE))

    return (quadrant, data)


def iter_data(ser):
    """Iterate through data.

    Note: This is blocking.

    Args:
        ser: Serial port.

    Yields:
        Tuple of (quadrant, data).
    """
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
            elif header == BOOTUP_HEADER:
                rospy.logwarn("Device was reset")


if __name__ == "__main__":
    rospy.init_node("nucleo", log_level=rospy.DEBUG)
    pub = rospy.Publisher("~signals", Signals, queue_size=1)

    # Get baudrate.
    baudrate = int(rospy.get_param("~baudrate", 115200))

    # Get whether 12 bit mode or not.
    TWELVE_BIT_MODE = bool(rospy.get_param("~twelve_bit_mode", True))
    rospy.loginfo("Assuming %d bit mode", 12 if TWELVE_BIT_MODE else 8)
    INT_SIZE = 16 if TWELVE_BIT_MODE else 8

    # Get buffersize.
    BUFFERSIZE = int(rospy.get_param("~buffersize", 1024))
    RAW_BUFFERSIZE = 2 * BUFFERSIZE if TWELVE_BIT_MODE else BUFFERSIZE

    with Serial("/dev/nucleo", baudrate=baudrate) as ser:
        rospy.loginfo("Waiting for device...")
        while not ser.readable() and not rospy.is_shutdown():
            pass
        rospy.loginfo("Found device")

        rospy.loginfo("Waiting for data...")
        signals = Signals()
        received = [0 for i in range(4)]
        previous_received_time = rospy.get_rostime()
        for quadrant, data in iter_data(ser):
            # Determine how long it has been since last packet.
            current_time = rospy.get_rostime()
            dt = current_time - previous_received_time
            previous_received_time = current_time

            rospy.logdebug("Received quadrant %d data", quadrant)

            # Reset if it has been too long, since packets might not match.
            if any(received) and dt > rospy.Duration(1):
                rospy.logwarn("Timeout %r, resetting...", dt.to_sec())
                received = [0 for i in received]

            # Verify no packet has been received multiple times.
            received[quadrant - 1] += 1
            if any(q > 1 for q in received):
                rospy.logwarn("Received inconsistent data %r", received)
                received = [0 for i in received]
                received[quadrant - 1] = 1

            # Set quadrant corresponding to packet.
            if quadrant == 1:
                signals.quadrant_1 = data
            elif quadrant == 2:
                signals.quadrant_2 = data
            elif quadrant == 3:
                signals.quadrant_3 = data
            elif quadrant == 4:
                signals.quadrant_4 = data

            # Send data if all packets were received.
            if all(q == 1 for q in received):
                pub.publish(signals)
                rospy.loginfo("Received ping")
                received = [0 for i in received]
