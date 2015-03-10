PKG = 'autonomy'
import roslib;  # This line is not needed with Catkin.

import sys
import unittest
import task
import x_axis_movement_timed_action
import go_to_object_action
import hail_marry_action
import set_cv_target_action
import buoy_task
import set_claw_state_action
import autonomy

from unittest import TestCase

class TestSample(unittest.TestCase):

    """def __init__(self,myautonomy):
      self.myautonomy = autonomy
"""

    def test_sanity(self):
	  self.assertEquals(8,8,"8!=8")
	
    def test_timed_movement(self):
      myauto = autonomy.Autonomy()
      here = "1,2,3,4"
      origin = myauto.get_transform(here)
      x_axis_movement_timed_action.XAxisMovementTimedAction(myauto,4,2)
      moved = myauto.get_transform(here)
      self.assertEquals(origin,moved)

    def test_absolute_movement(self):
      myauto = autonomy.Autonomy()
      go_to_object_action.GoToObjectAction(myauto,"0",0).execute()
      destination = myauto.get_transform("0")
      self.assertEquals(destination,(0.0,0.0))

    def test_hail_marry(self):
      myauto = autonomy.Autonomy()
      self.assertTrue(hail_marry_action.HailMarry(myauto, 0).execute)

    def test_cv(self):
      myauto = autonomy.Autonomy()
      self.assertTrue(set_cv_target_action.SetCVTargetAction(myauto,0))
   
    def test_stack(self):
      myauto = autonomy.Autonomy()
      task = buoy_task.BuoyTask(myauto).load_action_stack()
      testStack = []
      test_buoy_task = self.task.load_action_stack()
      my_first_ball = set_cv_target_action.SetCVTargetAction(myauto, firstBall) 
      my_second_ball = set_cv_target_action.SetCVTargetAction(myauto, secondBall) 
      go_to_my_first_ball = go_to_object_actiom.GoToObject(myauto,firstBall,0)
      go_to_my_second_ball = go_to_object_action.GoToObject(myauto,secondBall,0)  
      my_backward_movement_action = x_axis_movement_timed_action.XAxisMovementTimedAction(myauto,3,3)
      testStack.insert(0, my_first_ball)  
      testStack.insert(1, go_to_my_first_ball)
      testStack.insert(2, my_forward_movement_action)
      testStack.insert(3, my_backward_movement_action)
      testStack.insert(4, my_second_ball)
      testStack.insert(5, go_to_my_second_ball)
      testStack.insert(6, my_forward_movement_action)
      testStack.insert(7, my_backward_movement_action)  
      self.assertEquals(testStack,test_buoy_task,"1!=1")

    def test_claw_state(self):
      myauto = autonomy.Autonomy()
      self.assertTrue(set_claw_state_action.SetClawStateAction(myauto,"left").execute())


if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG,'FUCK', TestSample)


