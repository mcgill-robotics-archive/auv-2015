#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Read from Nucleo serial."""

import time
from serial import Serial

__author__ = "Anass Al-Wohoush"


with Serial("/dev/tty.usbmodem1413", baudrate=460800) as ser:
    start = [0, 0, 0, 0]
    while ser.readable():
        line = ser.readline()
        if line.startswith("DONE"):
            _, i = line.split()
            instance = int(i.split("\x00")[0])
            end = time.time()
            if start[instance]:
                print instance + 1, (end - start[instance]) / 1024. / 1000.
            start[instance] = end
        elif line.startswith("ALIVE"):
            # Print controller is still alive if it's been a while.
            if time.time() - max(start) > 2:
                print line,
        else:
            # Print everything else.
            print line,
