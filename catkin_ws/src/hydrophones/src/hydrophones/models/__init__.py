# -*- coding: utf-8 -*-

"""Hydrophone task models.

Attributes:
    audio: Incoming audio stream properties.
    dummy: Dummy pinger.
    env: Environment properties.
    mics: Microphone array.
    target: Target pinger.
    NUMBER_OF_MICS: Size of microphone array.
"""

__author__ = "Anass Al-Wohoush"
__version__ = "1.1.0"

from mic import Mic
from _env import env
from _audio import audio
from pinger import Pinger
from rospy import get_param
from _problem import Problem
from _solution import Solution

__all__ = [
    "Mic", "Pinger", "Problem", "Solution",
    "target", "dummy", "mics", "audio", "env",
    "NUMBER_OF_MICS"
]

# Determine target and dummy pingers
if env.is_practice:
    target = Pinger("practice")
    dummy = Pinger("competition")
else:
    target = Pinger("competition")
    dummy = Pinger("practice")

# Construct microphone array
_mic_ids = get_param(Mic.PARAM_PATH)
NUMBER_OF_MICS = len(_mic_ids)
mics = [Mic(id) for id in _mic_ids]

# Cleanup
del _mic_ids
del get_param
