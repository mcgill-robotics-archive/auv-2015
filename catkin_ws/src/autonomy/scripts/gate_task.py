#!/usr/bin/env python

import task
import autonomy
import task_controller
from abc import ABCMeta, abstractmethod

class Gate_Task(task.Task):
	def __init__(self, phase):
		self.phase = phase

	def execute(self):
		###########
		print self.phase