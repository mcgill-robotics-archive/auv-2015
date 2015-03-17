PKG = 'autonomy'
import roslib;  # This line is not needed with Catkin.

import sys
import unittest
import task
import x_axis_movement_timed_action
import go_to_object_action
import hail_marry_action
import set_cv_target_action
import set_claw_state_action
import bin_task
import autonomy

from unittest import TestCase

class bin_test(unittest.TestCase):

    """def __init__(self,myautonomy):
      self.myautonomy = autonomy
"""

    def test_sanity(self):
	  self.assertEquals(8,8,"8!=8")

    def test_hail_marry(self):
      myauto = autonomy.Autonomy()
      self.assertTrue(hail_marry_action.HailMarry(myauto, 0).execute)

    def test_cv(self):
      myauto = autonomy.Autonomy()
      self.assertTrue(set_cv_target_action.SetCVTargetAction(myauto,0))
   
    def test_stack(self):
      myauto = autonomy.Autonomy()
      task = bin_task.BinTask(myauto)
      test_bin_task = task.load_action_stack()
      testStack = [] 
      my_first_bin = set_cv_target_action.SetCVTargetAction(myauto,"firstBin")
      my_second_bin = set_cv_target_action.SetCVTargetAction(myauto,"secondBin")
      go_to_first_bin = go_to_object_action.GoToObjectAction(myauto,"firstBin",0)
      go_to_second_bin = go_to_object_action.GoToObjectAction(myauto,"secondBin",0)
      drop_left_bin = set_claw_state_action.SetClawStateAction(myauto,"left")
      drop_right_bin = set_claw_state_action.SetClawStateAction(myauto,"right")
      hail_marry_bin = hail_marry_action.HailMarry(myauto,"lastBin")
      testStack.insert(0, my_first_bin)
      testStack.insert(0, go_to_first_bin)
      testStack.insert(0, drop_left_bin)
      testStack.insert(0, my_second_bin)
      testStack.insert(0, go_to_second_bin)
      testStack.insert(0, drop_right_bin)
      testStack.insert(0, hail_marry_bin)
      self.assertEquals(testStack,test_bin_task)

    def test_claw_state(self):
      myauto = autonomy.Autonomy()
      self.assertTrue(set_claw_state_action.SetClawStateAction(myauto,"left").execute())


if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG,'FUCK', bin_test)


