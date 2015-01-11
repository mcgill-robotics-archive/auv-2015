# -*- coding: utf-8 -*-

"""Generates problem and expected solution."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0.0"

from tdoa import compute_tdoa
from hydrophones.models import Problem, Solution


def generate_problem(pinger, (x, y)=None):
    """Generates problem and expected solution given pinger.

    Args:
        pinger: Pinger to generate problem from.
        (x, y): Optional x, y coordinates of pinger in meters. By default uses
            the pinger's current coordinates.

    Returns:
        Tuple of (Problem, Solution) objects.

    Raises:
        KeyError: Pinger's coordinates not set.
    """
    if x and y:
        pinger.x = x
        pinger.y = y
    else:
        x = pinger.x
        y = pinger.y

    tdoa = compute_tdoa(pinger)
    problem = Problem(pinger, tdoa)
    solution = Solution(problem, (x, y))

    return problem, solution
