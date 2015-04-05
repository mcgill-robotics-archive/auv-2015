# -*- coding: utf-8 -*-

"""Index handler."""

from tornado.web import RequestHandler

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"


class IndexHandler(RequestHandler):

    """Index."""

    def get(self):
        """Renders index.html."""
        self.render("index.html")
