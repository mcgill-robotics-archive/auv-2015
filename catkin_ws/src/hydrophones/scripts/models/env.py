# -*- coding: utf-8 -*-

"""Task environment model."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"

import os
import rospy


class Environment(object):

    """Environment properties.

    Attributes:
        is_simulated: Whether the environment is simulated.
        speed: Speed of sound of environment in meters per second.
    """

    PARAM_PATH = "/hydrophones/env/"

    @property
    def is_simulated(self):
        """Whether the environment is simulated.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Environment.PARAM_PATH, "sim")
        if rospy.has_param(path):
            return bool(rospy.get_param(path))
        else:
            return False

    @is_simulated.setter
    def is_simulated(self, value):
        """Set if environment is simulated."""
        path = os.path.join(Environment.PARAM_PATH, "sim")
        rospy.set_param(path, bool(value))

    @property
    def speed(self):
        """Speed of sound of environment in meters per second.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Environment.PARAM_PATH, "speed")
        return float(rospy.get_param(path))

    @speed.setter
    def speed(self, value):
        """Set speed of sound in meters per second."""
        path = os.path.join(Environment.PARAM_PATH, "speed")
        rospy.set_param(path, float(value))

env = Environment()
