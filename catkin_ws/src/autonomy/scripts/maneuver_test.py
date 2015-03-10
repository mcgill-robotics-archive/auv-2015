#!/usr/bin/env python
PKG = 'autonomy'

import sys
import unittest
import maneuver_task

class TestManeuverTask(unittest.TestCase):
	def test_sanity(self):
		self.assertEquals(1, 1, "1 != 1")
	
	def test_load_stack(self):
		PathTask.load_action_stack = []
		self.assertTrue(PathTask.load_action_stack)
		PathTask.action_stack.insert()
		self.assertTrue(action_stack.insert != None)

	def test_phase(self):
		self.assertTrue(PathTask.phase > 0)
	
	def test_x_axis(self):
		test_velocity = 3
		XAxisMovementAction.execute(self)
		self.assertEquals(test_velocity = desired_velocity)

	def test_set_cv_target(self):
		set_target = SetCVTargetAction.execute(self)
		self.assertTrue(set_target)
		
	def test_go_to_object(self):
		bound = GoToObjectAction.out_of_bound(self)
		self.assertTrue(bound)
		x = 1
		y = 2
		z = 3
		roll = 4
		pitch = 5
		yaw = 6
		target = 'frame'
		GoToObjectAction.execute(self)
		self.assertEquals(position = (x, y, z, roll, pitch, yaw, target))

	def test_yaw_movement(self):
		test_velocity = 3
		YawAxisMovementAction.execute(self)
		self.assertEquals(test_velocity = desired_velocity)

	def test_y_axis(self): 
		test_velocity = 3
		YAxisMovementAction.execute(self)
		self.assertEquals(test_velocity = desired_velocity)

if __name__ == '__main__':

	import rostest
	rostest.rosrun(PKG, 'test_sanity', TestManeuverTask)
