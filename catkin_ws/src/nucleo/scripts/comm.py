#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Read from Nucleo serial."""

import time
import bitstring
from serial import Serial

__author__ = "Anass Al-Wohoush"


BUFFERSIZE = 6000

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
                end = time.time()
                _, i = header.strip("]").split()
                instance = int(i)
                if starts[instance]:
                    dt = (end - starts[instance]) / 1000.0 / BUFFERSIZE
                    print("dt: {}".format(dt))
                starts[instance] = end
                stream = bitstring.BitStream(bytes=raw)
                data = list(stream.read(16).uintle for i in range(BUFFERSIZE))
                print(sum(data) / len(data))
            else:
                print(ser.readline())

            # Reset header.
            header = ""
