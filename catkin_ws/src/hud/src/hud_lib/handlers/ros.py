# -*- coding: utf-8 -*-

"""ROS Handlers."""

import rospy
from std_msgs.msg import String
from tornado.websocket import WebSocketHandler

__author__ = "Anass Al-Wohoush"
__version__ = "1.0"


class ROSTopicHandler(WebSocketHandler):

    """ROS topic subscription socket."""

    def open(self, topic):
        """Opens socket and subscribes to topic."""
        rospy.Subscriber(topic, String, self.message_callback)

    def message_callback(self, msg):
        """Writes message data to client."""
        self.write_message(msg.data)
