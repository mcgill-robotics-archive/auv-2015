# -*- coding: utf-8 -*-

"""Pinger model."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"

import os
import rospy


class Pinger(object):

    """Pinger properties.

    Pinger IDs vary from 0 (target) to an arbitrary number of pingers.

    Attributes:
        id: Mic id (int).
        freq: Frequency of sound in Hz.
        pulse: Length of pulse in seconds.
        x: X coordinate relative to origin transducer in meters.
        y: Y coordinate relative to origin transducer in meters.
    """

    PARAM_PATH = "/hydrophones/pinger/"

    def __init__(self, id):
        """Construct Pinger object.

        Args:
            id: Mic id (int).
        """
        self.id = id

    def __repr__(self):
        """Return string representation of pinger."""
        return (
            "<Pinger"
            "freq: {pinger.freq} Hz"
            "pulse: {pinger.pulse:5.4f} s"
            "x: {pinger.x:+4.2f} m"
            "y: {pinger.y:+4.2f} m"
            ">"
        ).format(pinger=self)

    @property
    def freq(self):
        """Frequency in Hz.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, "freq")
        return float(rospy.get_param(path))

    @freq.setter
    def freq(self, value):
        """Set frequency in Hz."""
        path = os.path.join(Pinger.PARAM_PATH, "freq")
        rospy.set_param(path, float(value))

    @property
    def pulse(self):
        """Pulse length in seconds.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, "pulse")
        return float(rospy.get_param(path))

    @pulse.setter
    def pulse(self, value):
        """Set pulse length in seconds."""
        path = os.path.join(Pinger.PARAM_PATH, "pulse")
        rospy.set_param(path, float(value))

    @property
    def x(self):
        """X coordinate relative to origin microphone in meters.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, "x")
        return float(rospy.get_param(path))

    @x.setter
    def x(self, value):
        """Set X coordinate in meters."""
        path = os.path.join(Pinger.PARAM_PATH, "x")
        rospy.set_param(path, float(value))

    @property
    def y(self):
        """Y coordinate relative to origin microphone in meters.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, "y")
        return float(rospy.get_param(path))

    @y.setter
    def y(self, value):
        """Set Y coordinate in meters."""
        path = os.path.join(Pinger.PARAM_PATH, "y")
        rospy.set_param(path, float(value))

target = Pinger(0)
dummy = Pinger(1)
