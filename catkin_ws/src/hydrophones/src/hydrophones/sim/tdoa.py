# -*- coding: utf-8 -*-

"""Simulates TDOAs given pinger position."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0.0"

from numpy.linalg import norm
from hydrophones.models import env, mics, NUMBER_OF_MICS


def compute_tdoa(pinger):
    """Computes estimated microphone array's TDOAs given pinger's position.

    Args:
        pinger: Pinger of known position.

    Returns:
        List of time delay of arrivals relative to origin microphone in
        seconds. The list has as many elements as microphones, so the first
        TDOA is 0.
    """
    # Avoid inaccuracy due to change of pinger position during computation
    x, y = pinger.x, pinger.y

    # Determine times of arrival
    times_of_arrival = [
        norm([x - mics[i].x, y - mics[i].y]) / env.speed
        for i in range(NUMBER_OF_MICS)
    ]

    # Compute time differences of arrival
    tdoa = [
        times_of_arrival[i] - times_of_arrival[0]
        for i in range(NUMBER_OF_MICS)
    ]

    return tdoa
