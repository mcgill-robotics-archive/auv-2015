# -*- coding: utf-8 -*-

"""Pinger model."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0.0"

import os
import rospy


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

    def __repr__(self):
        """Return string representation of pinger."""
        return (
            "<Pinger"
            " id: {pinger.id}"
            " freq: {pinger.freq} Hz"
            " delay: {pinger.delay:4.3f} s"
            ">"
        ).format(pinger=self)

    @property
    def freq(self):
        """Pinger frequency in Hz.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "freq")
        return float(rospy.get_param(path))

    @freq.setter
    def freq(self, value):
        """Set pinger frequency in Hz."""
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "freq")
        rospy.set_param(path, float(value))

    @property
    def delay(self):
        """Pulse delay relative to competition pinger in seconds.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "delay")
        return float(rospy.get_param(path))

    @delay.setter
    def delay(self, value):
        """Set pulse delay relative to competition pinger in seconds."""
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "delay")
        rospy.set_param(path, float(value))

    @property
    def length(self):
        """Pulse length in seconds.

        This property is shared by all pingers.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, "length")
        return float(rospy.get_param(path))

    @length.setter
    def length(self, value):
        """Set pulse length in seconds.

        This property is shared by all pingers.
        """
        path = os.path.join(Pinger.PARAM_PATH, "length")
        rospy.set_param(path, float(value))

    @property
    def period(self):
        """Pulse period in seconds.

        This property is shared by all pingers.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, "period")
        return float(rospy.get_param(path))

    @period.setter
    def period(self, value):
        """Set pulse period in seconds.

        This property is shared by all pingers.
        """
        path = os.path.join(Pinger.PARAM_PATH, "period")
        rospy.set_param(path, float(value))

    @property
    def x(self):
        """X coordinate relative to origin microphone in meters.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "x")
        return float(rospy.get_param(path))

    @x.setter
    def x(self, value):
        """Set X coordinate in meters."""
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "x")
        rospy.set_param(path, float(value))

    @property
    def y(self):
        """Y coordinate relative to origin microphone in meters.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "y")
        return float(rospy.get_param(path))

    @y.setter
    def y(self, value):
        """Set Y coordinate in meters."""
        path = os.path.join(Pinger.PARAM_PATH, str(self.id), "y")
        rospy.set_param(path, float(value))
