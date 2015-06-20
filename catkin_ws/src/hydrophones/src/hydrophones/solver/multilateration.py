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

import numpy as np
from hydrophones.exceptions import CouldNotBeSolvedError

__author__ = "Anass Al-Wohoush"


def solve(mics, tdoa, speed):
    """Solves for 2D coordinates of pinger given list of time difference of
    arrivals.

    Args:
        mics: List of Mics. The first element must be the origin microphone.
        tdoa: List of time difference of arrivals in seconds.
        speed: Speed of sound in the medium in m/s.

    Returns:
        Estimated (X, Y) coordinates of pinger in meters.

    Raises:
        CouldNotBeSolvedError: Solution could not be found.
    """
    NUMBER_OF_MICS = len(mics)

    # Construct vectors.
    # A[m] = (2 * x[m]) / (v * t[m]) - (2 * x[1]) / (v * t[1])
    # B[m] = (2 * y[m]) / (v * t[m]) - (2 * y[1]) / (v * t[1])
    # C[m] = v * (t[m] - t[1]) - (x[m]^2 + y[m]^2) / (v * t[m])
    #      + (x[1]^2 + y[1]^2) / (v * t[1])
    A = [
        (2 / speed) * ((mics[i].x / tdoa[i]) - (mics[1].x / tdoa[1]))
        for i in range(2, NUMBER_OF_MICS)
    ]

    B = [
        (2 / speed) * ((mics[i].y / tdoa[i]) - (mics[1].y / tdoa[1]))
        for i in range(2, NUMBER_OF_MICS)
    ]

    C = [
        speed * (tdoa[i] - tdoa[1]) + (
            (mics[1].x ** 2 + mics[1].y ** 2) / tdoa[1] -
            (mics[i].x ** 2 + mics[i].y ** 2) / tdoa[i]
        ) / speed
        for i in range(2, NUMBER_OF_MICS)
    ]

    # Solve by QR.
    try:
        (x, y) = -np.linalg.solve(np.transpose([A, B]), C)
    except np.linalg.LinAlgError as e:
        # Matrix is either singular or impossible to solve.
        raise CouldNotBeSolvedError(e)

    # TODO(anassinator): Verify for overflow.
    pass

    return (x, y)
