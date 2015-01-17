PKG = 'test_roslaunch'
import roslib;  # This line is not needed with Catkin.

import sys
import unittest

from unittest import TestCase

class TestSample(TestCase):

	def test_sanity(self):
		self.assertEquals(8,8,"8!=8")

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'test_sanity', TestSample)


