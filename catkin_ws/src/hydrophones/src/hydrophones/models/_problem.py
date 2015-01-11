# -*- coding: utf-8 -*-

"""Formats hydrophones task problem and ROS message."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0.0"

from rospy import Time
from textwrap import dedent
from hydrophones.msg import TDOA

sequence = {}


class Problem(object):

    """Holds human readable hydrophones task problem and ROS message.

    Attributes:
        pinger: Pinger trying to solve for.
        tdoa: List of time delay of arrivals relative to origin microphone in
            seconds. The list must have as many elements as microphones, so the
            first TDOA is expected to be 0.
    """

    def __init__(self, pinger, tdoa):
        """Constructs Solution object.

        Args:
            pinger: Pinger trying to solve for.
            tdoa: List of time delay of arrivals relative to origin microphone
                in seconds. The list must have as many elements as microphones,
                so the first TDOA is expected to be 0.
        """
        self.seq = sequence.get(pinger.id, 0)
        self.pinger = pinger
        self.tdoa = tdoa
        sequence[pinger.id] = self.seq + 1

    def __str__(self):
        """Returns string representation of pinger solution."""
        return dedent(
            '''
            Problem {problem.pinger.id}
             seq: {problem.seq}
             tdoa: {problem.tdoa}
            '''
        ).format(problem=self)

    def msg(self):
        """Constructs hydrophones/TDOA ROS message.

        Raises:
            rospy.exceptions.ROSInitException: node not initialized properly.
        """
        tdoa_msg = TDOA()
        tdoa_msg.id = self.pinger.id

        # Header
        # TODO(anassinator): Add relevant header.frame_id.
        tdoa_msg.header.seq = self.seq
        tdoa_msg.header.stamp = Time.now()

        # Time differences of arrival
        tdoa_msg.data = self.tdoa

        return tdoa_msg
