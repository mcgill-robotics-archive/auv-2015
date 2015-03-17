PKG = 'autonomy'
import roslib;  # This line is not needed with Catkin.

import sys
import unittest
import task
import x_axis_movement_timed_action
import go_to_object_action
import set_cv_target_action
import torpedo_task
import fire_torpedo_action
import autonomy

from unittest import TestCase

class torpedo_test(unittest.TestCase):

    def test_sanity(self):
	  self.assertEquals(8,8,"8!=8")

    def test_cv(self):
      myauto = autonomy.Autonomy()
      self.assertTrue(set_cv_target_action.SetCVTargetAction(myauto,0))
   
    def test_stack(self):
      myauto = autonomy.Autonomy()
      task = torpedo_task.TorpedoTask(myauto)
      test_torpedo_task = task.load_action_stack()
      testStack = [] 
      
      self.assertEquals(testStack,test_torpedo_task)

    def test_torpedo_state(self):
      myauto = autonomy.Autonomy()
      self.assertTrue(fire_torpedo_action.FireTorpedoAction(myauto,"left").execute())


if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG,'FUCK', torpedo_test)

