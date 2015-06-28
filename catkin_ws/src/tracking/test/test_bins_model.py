import rosunit
from unittest import TestCase
from tracking.bins_model import BinsModel, PositionError
from math import pi, sin, cos, sqrt
import numpy as np


class TestBinsModel(TestCase):

    def test_calculate_direction_two_points(self):
        points = [[0, 0], [1, 0.1]]
        direction = BinsModel.calculate_direction(points)
        self.assertAlmostEqual(np.dot(direction, [-0.1, 1]), 0)

    def test_calculate_direction(self):
        points = [[2, 0], [-2, 0], [0, 1], [0, -1]]
        direction = BinsModel.calculate_direction(points)
        self.assertAlmostEqual(abs(direction[0]), 1)
        self.assertAlmostEqual(direction[1], 0)

    def test_calculate_bins_direction_multiple_bins(self):
        bins = [[[0, 0]], [[1, 0]], [[2, 0]]]
        direction = BinsModel.calculate_bins_direction(bins)
        self.assertAlmostEqual(abs(direction[0]), 1)
        self.assertAlmostEqual(direction[1], 0)

    def test_calculate_bins_direction_one_bin(self):
        bins = [[[0, 0], [0, 2], [1, 2], [1, 0]]]
        direction = BinsModel.calculate_bins_direction(bins)
        self.assertAlmostEqual(abs(direction[0]), 1)
        self.assertAlmostEqual(direction[1], 0)

    def test_directed_distance_offset(self):
        p1 = np.array([0, 0])
        p2 = np.array([10, 5])
        direction = np.array([1, 0])
        divisor = 2
        (quotient, remainder) = \
            BinsModel.directed_distance_offset(p1, p2, direction, divisor)
        self.assertEqual(quotient, 5)
        self.assertAlmostEqual(remainder[0], 0)
        self.assertAlmostEqual(remainder[1], 5)

    def test_calculate_bins_locations_raises_position_error(self):
        bins = np.array([[0, 0], [1.5, 0]])
        with self.assertRaises(PositionError):
            BinsModel.calculate_bins_locations(bins, np.array([1, 0]))

    def test_calculate_bins_locations(self):
        bins = np.array([[0, -0.1], [0, 1.1]])
        BinsModel.spacing = 1
        BinsModel.max_pos_error = 0.25
        (center, min_offset, max_offset) = \
            BinsModel.calculate_bins_locations(bins, np.array([0, 1]))
        self.assertEqual(min_offset, 0)
        self.assertEqual(max_offset, 1)
        self.assertAlmostEqual(center[0], 0)
        self.assertAlmostEqual(center[1], 0)

    def test_generate_configs_one_bin(self):
        bins = [[[0, 0], [0, 2], [1, 2], [1, 0]]]
        configs = BinsModel.generate_configs(bins, 4)
        self.assertEqual(len(configs), 4)

    def test_generate_configs_multiple_bins(self):
        BinsModel.spacing = 1
        bins = [[[0, 0]], [[1, 0]], [[2, 0]]]
        configs = BinsModel.generate_configs(bins, 4)
        self.assertEqual(len(configs), 2)

    def test_update_rotate(self):
        num_steps = 10
        BinsModel.spacing = 1
        BinsModel.yaw_dev = 1.01 * pi/(2 * num_steps)
        model = BinsModel([0, 0], [1, 0], 4)
        for theta in np.linspace(0, pi/2, num_steps + 1):
            bins = [[[0, 0]], [[cos(theta), sin(theta)]]]
            self.assertTrue(model.update(bins))
        self.assertFalse(model.update([[[0, 0]], [[0, 0]]]))

    def test_update_translate(self):
        num_steps = 10
        BinsModel.spacing = 1
        BinsModel.max_pos_error = 1.01 * sqrt(2) / num_steps
        model = BinsModel([0, 0], [1, 0], 4)
        for val in np.linspace(0, 1, num_steps + 1):
            self.assertTrue(model.update([[[val, val]], [[1 + val, val]]]))
        self.assertFalse(model.update([[[0, 0]], [[1, 0]]]))

    def test_update_shift(self):
        BinsModel.spacing = 1
        model = BinsModel([0, 0], [1, 0], 4)
        self.assertTrue(model.update([[[3, 0]], [[2, 0]]]))
        self.assertFalse(model.update([[[0, 0]], [[-1, 0]]]))


if __name__ == '__main__':
    rosunit.unitrun('test_tracking', 'test_bins_model', TestBinsModel)
