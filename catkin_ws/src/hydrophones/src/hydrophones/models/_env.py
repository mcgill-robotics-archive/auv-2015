# -*- coding: utf-8 -*-

"""Task environment model."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0.1"

import os
import rospy


class Environment(object):

    """Environment properties.

    Attributes:
        is_practice: Whether on practice side of the pool.
        is_simulated: Whether the environment is simulated.
        speed: Speed of sound of environment in meters per second.
    """

    PARAM_PATH = "/hydrophones/env/"

    @property
    def is_practice(self):
        """Whether on practice side of the pool.

        Raises:
            KeyError: ROS parameter is not set.
        """
        path = os.path.join(Environment.PARAM_PATH, "is_practice")
        return bool(rospy.get_param(path))

    @is_practice.setter
    def is_practice(self, value):
        """Set whether on practice side of the pool."""
        path = os.path.join(Environment.PARAM_PATH, "is_practice")
        rospy.set_param(path, bool(value))

    @property
    def is_simulated(self):
        """Whether the environment is simulated."""
        path = os.path.join(Environment.PARAM_PATH, "is_simulation")
        return bool(rospy.get_param(path, False))

    @is_simulated.setter
    def is_simulated(self, value):
        """Set whether environment is simulated."""
        path = os.path.join(Environment.PARAM_PATH, "is_simulation")
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
