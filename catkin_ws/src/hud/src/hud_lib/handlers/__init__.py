# -*- coding: utf-8 -*-

"""Handlers."""

from index import IndexHandler
from ros import ROSTopicHandler

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"


def get_handlers():
    return [
        (r"/", IndexHandler),
        (r"/ros/(.+)", ROSTopicHandler)
    ]
