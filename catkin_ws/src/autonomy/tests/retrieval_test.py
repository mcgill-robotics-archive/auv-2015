import sys
import unittest
import surface_action
import move_to_pinger_action
import placement_of_object_action
import retrieval_task
import autonomy

class TestCode(unittest.TestCase):
	def bareBones(self):
		self.assertEquals(1, 1, "1!=1")


	def testSurfaceAction(self):
		if (surface_action.surface(self, [0,0,0,0,0,0])):
			x = True
		self.assertTrue(x)


	def testCVTargetAction(self):
		desired_velocity = 0
		cv_target = None
		if (set_cv_target_action.execute(self) != Nil):
			x = True
		self.assertTrue(x)
	

	def testGrabberToggle(self):
		if (self.my_autonomy.toggle_grabber(self, "front", "open") = True and self.my_autonomy.toggle_grabber(self,"front","close") = True):
			x = True
		self.assertTrue(x)


	def testMoveToPingerAction(self):
		self.desired_velocity = 0
		self.assertTrue(move_to_pinger_action.execute(self))


	def testPlacementOfObjectAction(self):
		self.assertTrue(placement_of_object_action.execute(self))


	def testClawStateAction(self):
		if self.my_autonomy.drop_marker("bob") = "bob":
			x = True
		self.assertTrue(x)



	def testHydrophoneAction(self):
		if (self.my_autonomy.hyrophone_target_action(None) = None):
			self.assertTrue(True)
		else:
			self.assertTrue(False)
	


	def testPingerFound(self):
		target = None
		self.desired_target = target
		self.assertTrue(set_sonar_seek_target)

	
	def testGrabAction(self):
		self.my_autonomy.set_sonar_seek_target(self, None) = True
		self.assertTrue(grab_object_action.execute(self))



if __name__ == "__main__":
	import rostest
	rostest.rosrun('autonomy', 'test_retrieval', TestCode)