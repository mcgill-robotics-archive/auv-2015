import rosunit
from unittest import TestCase
from tracking.tracker import Tracker
from tracking.bins_model import BinsModel
from math import pi, sin, cos
import numpy as np

class TestTracker(TestCase):
    def test_tracker(self):
        BinsModel.spacing = 1
        t = Tracker(lambda x: BinsModel.generate_configs(x, 4), 5)
        t.update([[[0, 0], [0, 2], [1, 2], [1, 0]]])
        self.assertEqual(len(t.configurations), 4)
        t.update([[[3, 0], [3, 2], [4, 2], [4, 0]]])
        self.assertEqual(len(t.configurations), 1)

    def test_rotate(self):
        num_steps = 10
        BinsModel.spacing = 1
        BinsModel.yaw_dev = 1.01 * pi / (2 * num_steps) 
        t = Tracker(lambda x: BinsModel.generate_configs(x, 2), 5)
        for theta in np.linspace(0, pi/2, num_steps + 1):
            bins = [[[0, 0]], [[cos(theta), sin(theta)]]]
            t.update(bins)
            self.assertEqual(len(t.configurations), 1)
            self.assertAlmostEqual(np.dot(t.configurations[0].direction, bins[1][0]), 1)
        bins = [[[0, 0]], [[0, 0]]]
        t.update(bins)
        self.assertEqual(len(t.configurations), 0)

if __name__=='__main__':
    rosunit.unitrun('test_tracking', 'test_tracker', TestTracker)
