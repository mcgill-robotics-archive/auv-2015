#!/usr/bin/env python
"""@package docstring
This module is an abstract class to allow the interchanging of different actions
"""

from abc import ABCMeta, abstractmethod

class Action():
	__metaclass__ = ABCMeta
	my_autonomy = None

	def __init__(self, my_autonomy):
		self.my_autonomy = my_autonomy

	@abstractmethod
	def execute(self):	
		raise NotImplementedError()