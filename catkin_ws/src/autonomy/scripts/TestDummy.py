#!/usr/bin/env python
PKG = 'test_roslaunch'
import roslib;  # This line is not needed with Catkin.

import sys
import unittest

## A sample python unit test
class TestDummy(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_dummy', TestDummy)