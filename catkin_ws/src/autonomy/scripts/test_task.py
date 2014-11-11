#!/usr/bin/env python
"""@package docstring
This module is a task that can be used in place of a full routine for various
testing uses
"""
import task
import rospy
import autonomy
import task_controller
from abc import ABCMeta, abstractmethod

class Test_Task(task.Task):
	def __init__(self, phase, my_autonomy):
		self.phase = phase
		self.my_autonomy = my_autonomy


	def execute(self):
		if(self.phase == 1):
			self.phase1()

	def phase1(self):
		desired = [3, 0, 0, 0, 0, 0]
		while(True):
			#desired[0] = self.my_autonomy.filtered_depth
			self.my_autonomy.set_velocity(desired)
			rospy.sleep(1)