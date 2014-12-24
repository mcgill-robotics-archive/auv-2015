# -*- coding: utf-8 -*-

"""Audio model."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"

import os
import rospy


class Audio(object):

    """Incoming audio stream properties.

    Attributes:
        buffersize: Buffersize of incoming audio stream.
        fs: Sampling frequency in Hz.
    """

    PARAM_PATH = "/hydrophones/audio/"

    @property
    def buffersize(self):
        """Buffersize of incoming audio stream.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Audio.PARAM_PATH, "buffersize")
        return int(rospy.get_param(path))

    @buffersize.setter
    def buffersize(self, value):
        """Set buffersize of incoming audio stream."""
        path = os.path.join(Audio.PARAM_PATH, "buffersize")
        rospy.set_param(path, int(value))

    @property
    def fs(self):
        """Sampling frequency in Hz.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Audio.PARAM_PATH, "fs")
        return float(rospy.get_param(path))

    @fs.setter
    def fs(self, value):
        """Set sampling frequency in Hz."""
        path = os.path.join(Audio.PARAM_PATH, "fs")
        rospy.set_param(path, float(value))


audio = Audio()
