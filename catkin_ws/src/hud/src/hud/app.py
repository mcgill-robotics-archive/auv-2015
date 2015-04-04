# -*- coding: utf-8 -*-

"""McGill Robotics HUD."""

import random
import logging
import os.path
from tornado.options import define, options
from tornado.websocket import WebSocketHandler
from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.web import Application, RequestHandler

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"


class IndexHandler(RequestHandler):
    """Regular HTTP handler to serve the ping page"""
    def get(self):
        self.render("static/index.html")


class BatteryConnection(WebSocketHandler):
    def open(self):
        self.loop = PeriodicCallback(self.send_voltage, 100)
        self.loop.start()

    def on_message(self):
        pass

    def on_close(self):
        self.loop.stop()

    def send_voltage(self):
        self.write_message(str(random.random()))


class HUDApplication(Application):

    """HUD Application."""

    def __init__(self, debug):
        """Constructs HUDApplication."""
        handlers = [
            (r"/", IndexHandler),
            (r"/battery", BatteryConnection)
        ]
        settings = {
            "static_path": os.path.join(os.path.dirname(__file__), "static"),
            "debug": debug
        }
        super(HUDApplication, self).__init__(handlers, **settings)


def run(port, debug, host="0.0.0.0"):
    logging.critical("Running on http://{}:{}".format(host, port))
    HUDApplication(debug=debug).listen(port, host)
    IOLoop.instance().start()


if __name__ == "__main__":
    define("debug", default=False, help="debug mode", type=bool)
    define("port", default=8888, help="port", type=int)
    options.parse_command_line()

    run(options.port, options.debug)
