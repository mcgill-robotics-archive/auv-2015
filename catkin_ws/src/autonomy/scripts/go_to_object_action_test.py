PKG = 'autonomy'
import roslib;  # This line is not needed with Catkin.

import sys
import unittest
import task
import x_axis_movement_timed_action
import go_to_object_action
import autonomy

from unittest import TestCase

class go_to_object_action_test(unittest.TestCase):

    def test_sanity(self):
      self.assertEquals(8,8,"8!=8")

    def test_out_of_boudnds(self):
      myauto = autonomy.Autonomy()
      test_goto = go_to_object_action.GoToObjectAction(myauto,"test",0)
      self.assertTrue(test_goto.out_of_bound()) 

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG,'FUCK', go_to_object_action_test)


