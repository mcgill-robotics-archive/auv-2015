PKG = 'autonomy'
import rospy


import sys
import unittest
import threading


import surface_action
import move_to_pinger_action
import placement_of_object_action
import retrieval_task
import autonomy


from std_msgs.msg import String

MSG_DELAY=0.5

#######################################	
#######################################
class TestCode(unittest.TestCase):
#######################################
#######################################	


	def __init__(self, *args):
		super(TestCode, self).__init__(*args)



	####################################
	def setUp(self):
		rospy.init_node("test_retrieval_task")
		self.retrieval_pub = rospy.Publisher("retrieval task", String)

		
		rospy.Subscriber("retrieval task", String, retrieval_callback)
		rospy.sleep(MSG_DELAY)
	####################################


	####################################
	def publishMsgs(self, success, rate):
		r = rospy.Rate(rate)
		rospy.loginfo("-D- publishMsgs: I am experiencing %s" % success)
	####################################

	####################################	
	def retrievalCallback(self, msg):
		rospy.loginfo("-D- retrieval in progress? %s" % msg)
		self.status = msg
		self.time = rospy.Time.now()
	####################################	



	####################################
	def bareBones(self):
		rospy.loginfo("-D- SANITY TEST")
		myAutonomy = autonomy.Autonomy()
		self.assertEquals(1, 1, "1!=1")
	####################################


	###################################
	def testSurfaceAction(self):
		rospy.sleep(MSG_DELAY)
		rospy.loginfo("-D- testing surface action")
		myAutonomy = autonomy.Autonomy()
		self.assertTrue(surface_action.surface(self, [0,0,0,0,0,0]))
	###################################


	###################################
	def testCVTargetAction(self):
		rospy.loginfo("-D- testing CV target action")
		myAutonomy = autonomy.Autonomy()
		desired_velocity = 0
		cv_target = None
		self.assertTrue(set_cv_target_action.execute(self) != Nil)
	####################################


	####################################
	def testGrabberToggle(self):
		rospy.loginfo("-D- testing grabber toggle")
		myAutonomy = autonomy.Autonomy()
		self.assertTrue(self.myAutonomy.toggle_grabber(self, "front", "open") and self.myAutonomy.toggle_grabber(self,"front","close"))
	####################################


	####################################
	def testMoveToPingerAction(self):
		rospy.sleep(MSG_DELAY)
		rospy.loginfo("-D- testing pinger action")
		myAutonomy = autonomy.Autonomy()
		desired_velocity = [0,0,0,0,0,0]
		x = move_to_pinger_action(desired_velocity)
		self.assertTrue(x.execute())
	####################################


	####################################
	def testPlacementOfObjectAction(self):
		rospy.sleep(MSG_DELAY)
		rospy.loginfo("-D- testing placement of object action")
		myAutonomy = autonomy.Autonomy()
		x = placement_of_object_action(myAutonomy, 'test')
		self.assertTrue(x.execute())
	####################################



	####################################
	def testClawStateAction(self):
		rospy.sleep(MSG_DELAY)
		rospy.loginfo("-D- testing claw state action")
		myAutonomy = autonomy.Autonomy()
		self.assertEquals((myAutonomy.drop_marker("left"),myAutonomy.drop_marker("right")))
	####################################


	####################################
	def testHydrophoneAction(self):
		rospy.sleep(MSG_DELAY)
		rospy.loginfo("-D- testing hydrophone action")
		myAutonomy = autonomy.Autonomy()
		if (myAutonomy.hydrophone_target_action(None) == None):
			self.assertTrue(True)
		else:
			self.assertTrue(False)
	####################################
	

	####################################
	def testPingerFound(self):
		rospy.sleep(MSG_DELAY)
		rospy.loginfo("-D- testing pinger found action")
		myAutonomy = autonomy.Autonomy()
		target = None
		self.desired_target = target
		self.assertTrue(set_sonar_seek_target.execute())
	####################################


	####################################	
	def testGrabAction(self):
		rospy.sleep(MSG_DELAY)
		rospy.loginfo("-D- testing grabber action")
		myAutonomy = autonomy.Autonomy()
		
		self.assertTrue(grab_object_action.execute(self))
	####################################


	
	#Integration testing attempt#

	####################################
	def testStack(self):
		rospy.sleep(1)
		rospy.loginfo("-D- Integration Testing Commencing")

		myAutonomy = autonomy.Autonomy()
		myTask = retrieval_task.RetrievalTask(myAutonomy).load_action_stack()
		testStack = []
		testRetrievalTask = self.myTask.load_action_stack()

		
		retrieval_of_pinger_action = hydrophone_target_action.HydrophoneTargetAction(myAutonomy, "pinger")
		testStack.append(0, retrieval_of_pinger_action)

		
		move_to_pinger_action = go_to_object_action.GoToObjectAction(myAutonomy, "pinger", 2)
		testStack.append(0, move_to_pinger_action)

		
		surface_action = surface_action.SurfaceAction(myAutonomy, [0,0,3,0,0,0])
		testStack.append(0, surface_action)
		
		
		descend_action = z_axis_movement_action.ZAxisMovementAction(myAutonomy, [0,0,-3,0,0,0])
		testStack.append(0, descend_action)

		
		retrieval_of_object_action = set_sonar_seek_target_action.SetSonarSeekTargetAction(myAutonomy, targetObject)
		testStack.append(0, retrieval_of_object_action)

		
		grab_object_action = grab_object_action.GrabObjectAction(myAutonomy, "pole")
		testStack.append(0, placement_of_object_action)
		
		
		my_path_detection_action = set_cv_target.SetCVTargetAction(myAutonomy, pathColor)
		testStack.append(0, my_path_detection_action)
		
		
		go_towards_second_octagon_action = x_axis_movement_action.XAxisMovementAction(myAutonomy, 3)
		testStack.append(0, go_towards_second_octagon_action)
		
		
		testStack.append(0, surface_action)

		
		testStack.append(0, descend_action)

		
		placement_of_object_action = placement_of_object_action.PlacementOfObjectAction(myAutonomy, "pole")
		testStack.append(0, placement_of_object_action)

		
		placement_of_object_action = placement_of_object_action.PlacementOfObjectAction(myAutonomy)
		testStack.append(0, placement_of_object_action)

		
		retrieval_of_pinger_action = set_sonar_seek_target_action.SetSonarSeekTargetAction(myAutonomy, pinger)
		testStack.append(0, retrieval_of_pinger_action)

		
		move_to_pinger_action = go_to_object_action.GoToObjectAction(myAutonomy, "pinger", 2)
		testStack.append(0, move_to_pinger_action)

		
		placement_of_object_action = placement_of_object_action.PlacementOfObjectAction(myAutonomy)
		testStack.append(0, placement_of_object_action)

		
		surface_action = z_axis_movement_action.ZAxisMovementAction(myAutonomy, [0,0,3,0,0,0])
		testStack.append(0, surface_action)


		self.assertEquals(testStack, testRetrievalTask, "1!=1")
	####################################



if __name__ == "__main__":
	import rostest
	rospy.loginfo("Testing Retrieval Task Started")
	rostest.rosrun(PKG, 'test_retrieval', TestCode)