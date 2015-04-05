# -*- coding: utf-8 -*-

"""Handlers."""

from index import IndexHandler
from tornado.web import StaticFileHandler

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"


def get_handlers(settings):
    return [
        (r"/", IndexHandler),
        (r"/static/(.*)", StaticFileHandler, dict(path=settings["static_path"]))
    ]
