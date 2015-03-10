#!/usr/bin/env python
PKG = 'autonomy'

import sys
import unittest
import x_axis_movement_action
import path_task

class TestPathTask(unittest.TestCase):

	def test_sanity(self):
		self.assertEquals(1, 1, "1 != 1")
	
	def test_x_axis_action(self):
		self.desired_velocity = 3
		bool = x_axis_movement_action.XAxisMovementAction.execute(x_axis_movement_action.XAxisMovementAction(self, self.desired_velocity))
		if (bool == True):
			if (self.velocity_array == [3, 0, 0, 0, 0, 0]):
				x = True
		self.assertTrue(x)
			
	
"""	def test_load_stack(self):
		PathTask.load_action_stack = []
		self.assertTrue(PathTask.load_action_stack)
		PathTask.action_stack.insert(self)
		self.assertTrue(action_stack.insert != None)

	def test_phase(self):
		self.assertTrue(PathTask.phase > 0)"""
	

"""	def test_set_cv_target(self):
		set_target = SetCVTargetAction.execute(self)
		self.assertTrue(set_target)
		
	def test_go_to_object(self):
		x = 1
		y = 2
		z = 3
		roll = 4
		pitch = 5
		yaw = 6
		target = 'frame'
		GoToObjectAction.execute(self)
		self.assertEquals(position = (x, y, z, roll, pitch, yaw, target))"""


if __name__ == "__main__":

	import rostest
	rostest.rosrun(PKG, 'test_sanity', TestPathTask)
