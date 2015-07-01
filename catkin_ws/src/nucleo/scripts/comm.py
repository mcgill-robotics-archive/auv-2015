#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Read from Nucleo serial."""

from serial import Serial

__author__ = "Anass Al-Wohoush"


with Serial("/dev/tty.usbmodem1423", baudrate=115200) as ser:
    while ser.readable():
        c = ser.read(1)
        if c != "a":
            print "ERR", c
