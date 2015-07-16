#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Read from Nucleo serial."""

from __future__ import print_function

import time
import bitstring
from serial import Serial

__author__ = "Anass Al-Wohoush"

TWELVE_BIT_MODE = False
LOOPS = 1000.0
BUFFERSIZE = 6000

RAW_BUFFERSIZE = 2 * BUFFERSIZE if TWELVE_BIT_MODE else BUFFERSIZE
INT_SIZE = 16 if TWELVE_BIT_MODE else 8

if TWELVE_BIT_MODE:
    print("Assuming 12 bit mode...")
else:
    print("Assuming 8 bit mode...")

headers = (
    "[DATA 0]",
    "[DATA 1]",
    "[DATA 2]",
    "[DATA 3]",
    "[DEBUG]",
    "[FATAL]"
)


if __name__ == "__main__":
    starts = [0, 0, 0, 0]

    with Serial("/dev/tty.usbmodem1413", baudrate=230400) as ser:
        print("Initializing...")
        while not ser.readable():
            pass

        # Synchronize.
        print("Synchronizing...")
        while ser.read(1) != "[":
            pass

        header = "["

        print("Waiting for data...")
        while ser.readable():
            skipped = 0
            while True:
                # Read until next line.
                header += ser.readline().strip()

                # Verify if header is valid.
                if header in headers:
                    if skipped:
                        print("SKIPPED {}".format(skipped))
                    break

                # Otherwise reset.
                header = ""

            # Process data.
            print(header, end=" ")
            if "DATA" in header:
                # Stop timer.
                end = time.time()

                # Get data.
                raw = ser.read(RAW_BUFFERSIZE)

                # Determine ADC.
                _, i = header.strip("]").split()
                instance = int(i)

                # Convert to array.
                stream = bitstring.BitStream(bytes=raw)
                data = list(
                    stream.read(INT_SIZE).uintle
                    for i in range(BUFFERSIZE))

                # Determine sampling frequency.
                if starts[instance]:
                    fs = 1 / ((end - starts[instance]) / LOOPS / BUFFERSIZE)
                    print("fs: {}".format(fs), end=" ")

                # Set timer.
                starts[instance] = end

                # Print mean.
                mean = sum(data) / float(len(data))
                print("mean: {}".format(mean))
            else:
                print(ser.readline().strip())

            # Reset header.
            header = ""
