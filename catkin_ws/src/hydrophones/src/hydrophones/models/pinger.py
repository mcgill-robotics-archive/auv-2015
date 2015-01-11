# -*- coding: utf-8 -*-

"""Pinger model."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.1.0"

import os
from textwrap import dedent
from rospy import get_param, set_param


class Pinger(object):

    """Pinger properties.

    Pinger IDs are as in parameter server.

    Attributes:
        id: Pinger ID.
        freq: Frequency of sound in Hz.
        period: Pulse period in seconds.
        length: Pulse length in seconds.
        delay: Pulse delay relative to competition pinger in seconds.
        x: X coordinate relative to origin transducer in meters.
        y: Y coordinate relative to origin transducer in meters.
    """

    PARAM_PATH = "/hydrophones/pinger/"

    def __init__(self, id):
        """Construct Pinger object.

        Args:
            id: Pinger ID.
        """
        self.id = id

    def __str__(self):
        """Return string representation of pinger."""
        return dedent(
            '''
            Pinger {pinger.id}
             freq: {pinger.freq} Hz
             delay: {pinger.delay:4.3f} s
            '''
        ).format(sol=self)

    @property
    def freq(self):
        """Pinger frequency in Hz.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "freq")
        return float(get_param(path))

    @freq.setter
    def freq(self, value):
        """Set pinger frequency in Hz."""
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "freq")
        set_param(path, float(value))

    @property
    def delay(self):
        """Pulse delay relative to competition pinger in seconds.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "delay")
        return float(get_param(path))

    @delay.setter
    def delay(self, value):
        """Set pulse delay relative to competition pinger in seconds."""
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "delay")
        set_param(path, float(value))

    @property
    def length(self):
        """Pulse length in seconds.

        This property is shared by all pingers.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, "length")
        return float(get_param(path))

    @length.setter
    def length(self, value):
        """Set pulse length in seconds.

        This property is shared by all pingers.
        """
        path = os.path.join(Pinger.PARAM_PATH, "length")
        set_param(path, float(value))

    @property
    def period(self):
        """Pulse period in seconds.

        This property is shared by all pingers.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, "period")
        return float(get_param(path))

    @period.setter
    def period(self, value):
        """Set pulse period in seconds.

        This property is shared by all pingers.
        """
        path = os.path.join(Pinger.PARAM_PATH, "period")
        set_param(path, float(value))

    @property
    def x(self):
        """X coordinate relative to origin microphone in meters.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "x")
        return float(get_param(path))

    @x.setter
    def x(self, value):
        """Set X coordinate in meters."""
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "x")
        set_param(path, float(value))

    @property
    def y(self):
        """Y coordinate relative to origin microphone in meters.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "y")
        return float(get_param(path))

    @y.setter
    def y(self, value):
        """Set Y coordinate in meters."""
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "y")
        set_param(path, float(value))
