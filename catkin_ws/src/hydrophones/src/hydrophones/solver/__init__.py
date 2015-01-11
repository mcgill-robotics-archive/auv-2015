# -*- coding: utf-8 -*-

"""Solves for location of pinger given time delay of arrivals."""

__author__ = "Anass Al-Wohoush"
__version__ = "1.0.0"

from rospy import get_param
import importlib as __importer
from hydrophones.models import Solution

__all__ = ["solve"]


def __get_solver(module):
    """Return and import solver method from submodule.

    Args:
        module: Module to solve with.

    Returns:
        Executable solve() method.
    """
    module = '.'.join(["hydrophones", "solver", module])
    return __importer.import_module(module).solve


def solve(problem, method=None):
    """Solve hydrophone task given problem.

    Args:
        problem: Problem to solve.
        method: Method to solve problem with. Must be a valid sub-module name.
            By default, selects method set in '/hydrophones/method' parameter.

    Returns:
        Solution object.
    """
    # Select method
    if method:
        solver = __get_solver(method)
    else:
        solver = __get_solver(get_param("/hydrophones/method"))

    sol = solver(problem.tdoa)

    return Solution(problem, sol)
