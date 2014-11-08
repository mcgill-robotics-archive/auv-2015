#!/usr/bin/env python

import autonomy
import task_controller
from abc import ABCMeta, abstractmethod

class Task():
	__metaclass__ = ABCMeta
	phase = -1

	def __init__(self, phase):
		self.phase = phase

	@abstractmethod
	def execute(self):	
		raise NotImplementedError()