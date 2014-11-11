#!/usr/bin/env python
"""@package docstring
This module is an abstract class to allow the interchanging of different tasks
"""
import autonomy
import task_controller
from abc import ABCMeta, abstractmethod

class Task():
	__metaclass__ = ABCMeta
	phase = -1
	my_autonomy = None

	def __init__(self, phase, my_autonomy):
		self.phase = phase
		self.my_autonomy = my_autonomy

	@abstractmethod
	def execute(self):	
		raise NotImplementedError()