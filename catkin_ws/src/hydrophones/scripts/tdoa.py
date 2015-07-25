#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Hydrophones signal analysis."""

import rospy
from hydrophones import Mic
from auv_msgs.msg import Signals
from hydrophones.gccphat import estimate
from hydrophones.multilateration import solve
from hydrophones.freq import get_frequency_energy

FS = 972972.97297

mics = [
    Mic((0.00, 0.00, 0.00)),
    Mic((-0.013, 0.00, 0.00)),
    Mic((-0.013, -0.013, 0.00)),
    Mic((0.00, -0.013, 0.00))
]


def analyze(msg):
    print(get_frequency_energy(msg.quadrant_1, FS, 28000))
    tdoa = [
        estimate(msg.quadrant_1, msg.quadrant_1, FS),
        estimate(msg.quadrant_1, msg.quadrant_2, FS),
        estimate(msg.quadrant_1, msg.quadrant_3, FS),
        estimate(msg.quadrant_1, msg.quadrant_4, FS)
    ]
    for dt in tdoa:
        print(dt)
    x, y = solve(mics, tdoa, 1500.0)
    print(x, y)
    print("---")


if __name__ == "__main__":
    rospy.init_node("tdoa")
    rospy.Subscriber("/nucleo/signals", Signals, analyze, queue_size=10)
    rospy.spin()
