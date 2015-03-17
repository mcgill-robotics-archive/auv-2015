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
import autonomy

from unittest import TestCase

class buoy_test(unittest.TestCase):

    def test_sanity(self):
      self.assertEquals(8,8,"8!=8")

    def test_cv(self):
      myauto = autonomy.Autonomy()
      self.assertTrue(set_cv_target_action.SetCVTargetAction(myauto,0))
   
    def test_stack(self):
      myauto = autonomy.Autonomy()
      task = buoy_task.BuoyTask(myauto)
      testStack = []
      test_buoy_task = task.load_action_stack()
      my_first_ball = set_cv_target_action.SetCVTargetAction(myauto, "firstBall") 
      my_second_ball = set_cv_target_action.SetCVTargetAction(myauto, "secondBall") 
      go_to_my_first_ball = go_to_object_action.GoToObjectAction(myauto,"firstBall",0)
      go_to_my_second_ball = go_to_object_action.GoToObjectAction(myauto,"secondBall",0)  
      my_backward_movement_action = x_axis_movement_timed_action.XAxisMovementTimedAction(myauto,3,3)
      testStack.insert(0, my_first_ball)  
      testStack.insert(1, go_to_my_first_ball)
      testStack.insert(2, my_backward_movement_action)
      testStack.insert(3, my_second_ball)
      testStack.insert(4, go_to_my_second_ball)
      testStack.insert(5, my_backward_movement_action)  

      self.assertEquals(testStack,test_buoy_task)

    def test_x_axis_timed(self):
      myauto = autonomy.Autonomy()
      x_timed = x_axis_movement_timed_action.XAxisMovementTimedAction(myauto,1,3)
      self.assertTrue(x_timed.execute())

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG,'FUCK', buoy_test)


