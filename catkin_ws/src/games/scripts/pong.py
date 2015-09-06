#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""McGill Robotics pong game."""

import rospy
from games import pong

__author__ = "Anass Al-Wohoush"

if __name__ == "__main__":
    rospy.init_node("pong")
    height = rospy.get_param("~height", 7)
    width = rospy.get_param("~width", 13)
    players = rospy.get_param("~players", 1)
    blinky = rospy.get_param("~blinky", None)
    pong.play(height=height, width=width, players=players, blinky=blinky)
