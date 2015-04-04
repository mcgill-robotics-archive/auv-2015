# -*- coding: utf-8 -*-

"""McGill Robotics HUD."""

import rospy
from handlers import get_handlers
from tornado.ioloop import IOLoop
from tornado.web import Application
from tornado.options import define, options

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"


class HUDApplication(Application):

    """HUD Application."""

    def __init__(self, **kwargs):
        """Constructs HUDApplication."""
        super(HUDApplication, self).__init__(get_handlers(), **kwargs)


def run(port, debug, host="0.0.0.0"):
    rospy.loginfo("Running on http://{}:{}".format(host, port))
    app = HUDApplication(debug=debug)
    app.listen(port, host)
    IOLoop.instance().start()


if __name__ == "__main__":
    define("debug", default=False, help="debug mode", type=bool)
    define("port", default=8888, help="port", type=int)
    options.parse_command_line()
    run(options.port, options.debug)
