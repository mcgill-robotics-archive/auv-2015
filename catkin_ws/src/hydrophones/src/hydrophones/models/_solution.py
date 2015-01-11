# -*- coding: utf-8 -*-

"""Formats pinger solution and ROS message."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0.0"

from rospy import Time
from textwrap import dedent
from numpy.linalg import norm
from numpy import arctan2, rad2deg
from hydrophones.msg import Pinger


class Solution(object):

    """Holds human readable pinger solution and ROS message.

    Attributes:
        problem: Problem solved.
        x: X coordinate from origin microphone in meters.
        y: Y coordinate from origin microphone in meters.
        distance: Distance from origin microphone in meters.
        theta: Heading in radians.
        degrees: Heading in degrees.
    """

    def __init__(self, problem, (x, y)):
        """Constructs Solution object.

        Args:
            problem: Problem solved.
            (x, y): Tuple of x, y coordinates from origin microphone in meters.
        """
        self.problem = problem
        self.x = x
        self.y = y
        self.distance = norm([x, y])
        self.theta = arctan2(y, x)

    def __str__(self):
        """Returns string representation of pinger solution."""
        return dedent(
            '''
            Solution {sol.problem.pinger.id}
             x: {sol.x:+5.2f} m
             y: {sol.y:+5.2f} m
             distance: {sol.distance:5.2f} m
             degrees: {sol.degrees:5.2f}
            '''
        ).format(sol=self)

    @property
    def degrees(self):
        """Returns heading in degrees."""
        return rad2deg(self.theta)

    def msg(self):
        """Constructs hydrophones/Pinger ROS message.

        Raises:
            rospy.exceptions.ROSInitException: node not initialized properly.
        """
        pinger_msg = Pinger()
        pinger_msg.id = self.problem.pinger.id

        # Header
        # TODO(anassinator): Add relevant header.frame_id.
        pinger_msg.header.seq = self.problem.seq
        pinger_msg.header.stamp = Time.now()

        # Cartesian coordinates
        # TODO(anassinator): Add depth to point.z.
        pinger_msg.point.x = self.x
        pinger_msg.point.y = self.y

        # Polar coordinates
        pinger_msg.distance = self.distance
        pinger_msg.theta = self.theta

        return pinger_msg
