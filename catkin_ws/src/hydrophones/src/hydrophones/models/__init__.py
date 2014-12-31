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
__version__ = "1.0.0"

import rospy
from mic import Mic
from audio import Audio
from pinger import Pinger
from env import Environment

# Properties
audio = Audio()
env = Environment()

# Determine target and dummy pingers
if env.is_practice:
    target = Pinger('practice')
    dummy = Pinger('competition')
else:
    target = Pinger('competition')
    dummy = Pinger('practice')

# Construct microphone array
_mic_ids = rospy.get_param(Mic.PARAM_PATH)
NUMBER_OF_MICS = len(_mic_ids)
mics = [Mic(id) for id in _mic_ids]

# Cleanup
del rospy
del _mic_ids
