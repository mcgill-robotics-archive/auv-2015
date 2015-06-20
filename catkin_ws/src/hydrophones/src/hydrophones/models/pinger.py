# -*- coding: utf-8 -*-

"""Pinger model."""

__author__ = "Anass Al-Wohoush"


class Pinger(object):

    """Pinger object."""

    def __init__(self, (x, y, z), frequency):
        """Constructs Pinger instance.

        Args:
            (x, y, z): Coordinates of pinger relative to origin mic in meters.
            frequency: Frequency of ping in Hz.
        """
        self.x = x
        self.y = y
        self.z = z
        self.freq = frequency
