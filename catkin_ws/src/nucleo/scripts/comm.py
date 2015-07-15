#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Read from Nucleo serial."""

import bitstring
from serial import Serial

__author__ = "Anass Al-Wohoush"


BUFFERSIZE = 1024

headers = (
    "[DATA0]",
    "[DATA1]",
    "[DATA2]",
    "[DATA3]",
    "[DEBUG]",
    "[FATAL]"
)


if __name__ == "__main__":
    with Serial("/dev/tty.usbmodem1413", baudrate=115200) as ser:
        print("Initializing...")
        while not ser.readable():
            pass

        # Synchronize.
        print("Synchronizing...")
        while ser.read(1) != "[":
            pass

        header = "["

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
            print(header)
            if "DATA" in header:
                raw = ser.read(2 * BUFFERSIZE)
                stream = bitstring.BitStream(bytes=raw)
                data = list(stream.read(16).uintle for i in range(BUFFERSIZE))
                print(sum(data) / len(data))
            else:
                print(ser.readline())

            # Reset header.
            header = ""
