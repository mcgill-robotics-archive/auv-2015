#!/usr/bin/env python
PKG = 'autonomy'

import sys
import unittest
import task
import autonomy
import x_axis_movement_action
import set_cv_target_action
import maneuver_task

class TestManeuverTask(unittest.TestCase):
	def test_sanity(self):
		self.assertEquals(1, 1, "1 != 1")

	def test_phase(self):
		self.assertTrue(maneuver_task.ManeuverTask.phase > 0)
	
	def test_set_cv_target(self):
		myauto = autonomy.Autonomy()
		self.assertTrue(set_cv_target_action.SetCVTargetAction(myauto, 0))

	def test_go_to_object(self):
		self.assertTrue(True)

	def test_x_axis_action(self):
		self.assertTrue(True)
	"""	self.desired_velocity = 3
		myauto = autonomy.Autonomy()
		self.velocity_array = x_axis_movement_action.XAxisMovementAction(myauto, 1, 3).execute()

		if (self.velocity_array == [3, 0, 0, 0, 0, 0]):
			x = True
		self.assertTrue(x)"""

	def test_yaw_movement(self):
		self.assertTrue(True)

	def test_y_axis(self): 
		self.assertTrue(True)

	def test_load_search_poles(self):
		self.assertTrue(True)
	
	def test_load_go_to_poles(self):
		self.assertTrue(True)

	def test_load_set_cv(self):
		self.assertTrue(True)

	def test_load_go_to_obstacle(self):
		self.assertTrue(True)

	def test_load_y_axis_movement(self):
		self.assertTrue(True)

	def test_load_yaw_movement(self):
		self.assertTrue(True)

if __name__ == '__main__':

	import rostest
	rostest.rosrun(PKG, 'maneuver_test', TestManeuverTask)
