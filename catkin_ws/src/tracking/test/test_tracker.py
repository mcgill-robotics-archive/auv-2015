import rosunit
from unittest import TestCase
from tracking.tracker import Tracker
from tracking.bins_model import BinsModel

class TestTracker(TestCase):
    def test_tracker(self):
        BinsModel.spacing = 1
        t = Tracker(lambda x: BinsModel.generate_configs(x, 4), 5)
        t.update([[[0, 0], [0, 2], [1, 2], [1, 0]]])
        self.assertEqual(len(t.configurations), 4)
        t.update([[[3, 0], [3, 2], [4, 2], [4, 0]]])
        self.assertEqual(len(t.configurations), 1)

if __name__=='__main__':
    rosunit.unitrun('test_tracking', 'test_tracker', TestTracker)
