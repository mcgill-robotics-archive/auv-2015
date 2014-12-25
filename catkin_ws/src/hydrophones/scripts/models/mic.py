# -*- coding: utf-8 -*-

"""Microphone model."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"

import os
import rospy


class Mic(object):

    """Microphone properties.

    Microphone IDs range from 0 (origin) to the number of microphones in the
    array exclusively. Origin microphone is placed at (0,0).

    Attributes:
        id: Mic id (int).
        x: X coordinate relative to origin microphone in meters.
        y: Y coordinate relative to origin microphone in meters.
    """

    PARAM_PATH = "/hydrophones/mic/"

    def __init__(self, id):
        """Construct Mic object.

        Args:
            id: Mic id (int).
        """
        self.id = id
        self._path_x = os.path.join(Mic.PARAM_PATH, str(self.id), 'x')
        self._path_y = os.path.join(Mic.PARAM_PATH, str(self.id), 'y')

    def __repr__(self):
        """Return string representation of microphone."""
        return "<Mic x: {mic.x:+4.2f} m y: {mic.y:+4.2f} m>".format(mic=self)

    @property
    def x(self):
        """X coordinate relative to origin microphone in meters.

        Raises:
            KeyError: ROS parameter is not set.
        """
        return float(rospy.get_param(self._path_x))

    @x.setter
    def x(self, value):
        """Set X coordinate in meters."""
        rospy.set_param(self._path_x, float(value))

    @property
    def y(self):
        """Y coordinate relative to origin microphone in meters.

        Raises:
            KeyError: ROS parameter is not set.
        """
        return float(rospy.get_param(self._path_y))

    @y.setter
    def y(self, value):
        """Set Y coordinate in meters."""
        rospy.set_param(self._path_y, float(value))


# Microphone array
NUMBER_OF_MICS = 4
mics = [Mic(id) for id in range(NUMBER_OF_MICS)]
