PKG = 'test_function'
import roslib;  # This line is not needed with Catkin.

import sys
import unittest

from unittest import TestCase

class TestSample(TestCase)

	def test_sanity(self)
		self.assertEqual(8,4+4, ¨4 + 4 != 8¨)

if __name__ == '__main__':
	import rosunit
	rosunit.unitrun(PKG, 'test_sanity', TestSample)


