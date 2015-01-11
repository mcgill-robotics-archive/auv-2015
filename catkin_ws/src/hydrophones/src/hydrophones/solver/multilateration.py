# -*- coding: utf-8 -*-

"""Through multilateration, solves for location of pinger given TDOAs.

This method requires at least three TDOAs (i.e. four hydrophones) in order to
work, and works as is given more. Accuracy increases as the number of TDOAs
increases.

Unfortunately, the accuracy of the estimated distance to the pinger is heavily
dependent on the accuracy of the TDOAs, hence can be very unustable. The
estimated heading however is very accurate and is sufficient to solve the task.

The algorithm is adapted to two-dimensional space from
http://en.wikipedia.org/wiki/Multilateration
"""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0.0"

import rospy
import numpy as np
from hydrophones.models import env, mics, NUMBER_OF_MICS


def solve(tdoa):
    """Solves for X, Y coordinates given list of TDOAs with multilateration.

    Args:
        tdoa: List of time delay of arrivals relative to origin microphone in
            seconds. The list must have as many elements as microphones, so the
            first TDOA is expected to be 0.

    Returns:
        Tuple of (X, Y) coordinates of pinger in meters.
    """
    # Construct vectors
    A = [
        (2 / env.speed) * ((mics[i].x / tdoa[i]) - (mics[1].x / tdoa[1]))
        for i in range(2, NUMBER_OF_MICS)
    ]

    B = [
        (2 / env.speed) * ((mics[i].y / tdoa[i]) - (mics[1].y / tdoa[1]))
        for i in range(2, NUMBER_OF_MICS)
    ]

    C = [
        env.speed * (tdoa[i] - tdoa[1])
        - (mics[i].x ** 2 + mics[i].y ** 2) / (env.speed * tdoa[i])
        + (mics[1].x ** 2 + mics[1].y ** 2) / (env.speed * tdoa[1])
        for i in range(2, NUMBER_OF_MICS)
    ]

    # Solve by QR
    (x, y) = -np.linalg.solve(np.transpose([A, B]), C)

    # Verify for overflow due to inaccuracy
    # Currently only valid for 4 microphones in rectangular array.
    # TODO(anassinator): Generalize for any microphone array.
    if NUMBER_OF_MICS == 4 and True:
        x_axis_delay = tdoa[1]
        y_axis_delay = tdoa[3]
        WIDTH, HEIGHT = mics[1].x, mics[3].y
        if x_axis_delay > 0 and y_axis_delay > 0:
            if x > WIDTH / 2 and y > HEIGHT / 2:
                x = -np.abs(x)
                y = -np.abs(y)
                rospy.logwarn("CORRECTED FOR QUADRANT III")
        elif x_axis_delay < 0 and y_axis_delay < 0:
            if x < WIDTH / 2 and y < HEIGHT / 2:
                x = np.abs(x)
                y = np.abs(y)
                rospy.logwarn("CORRECTED FOR QUADRANT I")
        elif x_axis_delay < 0 and y_axis_delay > 0:
            if x < WIDTH / 2 and y > HEIGHT / 2:
                x = np.abs(x)
                y = -np.abs(y)
                rospy.logwarn("CORRECTED FOR QUADRANT IV")
        elif x_axis_delay > 0 and y_axis_delay < 0:
            if x > WIDTH / 2 and y < HEIGHT / 2:
                x = -np.abs(x)
                y = np.abs(y)
                rospy.logwarn("CORRECTED FOR QUADRANT II")

    return (x, y)
