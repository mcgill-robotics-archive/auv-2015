#!/usr/bin/python
# -*- coding: utf-8 -*-

"""McGill Robotics HUD."""

import rospy
from hud_lib import app
from tornado.options import define, options

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"


if __name__ == "__main__":
    define("debug", default=False, help="debug mode", type=bool)
    define("port", default=8888, help="port", type=int)
    options.parse_command_line()

    rospy.init_node("hud")
    app.run(options.port, options.debug)
