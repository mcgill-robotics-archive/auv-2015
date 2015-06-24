from tracking.bins_model import BinsModel
from math import pi, sin, cos
import numpy as np
def test_update_translate():
    import pdb; pdb.set_trace()
    num_steps = 10
    BinsModel.spacing = 1
    BinsModel.max_pos_error = 1.01 / num_steps
    model = BinsModel([0, 0], [1, 0], 4)
    for val in np.linspace(0, 1, num_steps + 1):
        model.update([[[val, val]], [[1 + val, val]]])
    model.update([[[0, 0]], [[1, 0]]])


def test_update_rotate(self):
    import pdb; pdb.set_trace()
    num_steps = 10
    BinsModel.spacing = 1
    BinsModel.yaw_dev = 1.01 * pi/(2 * num_steps)
    model = BinsModel([0, 0], [1, 0], 4)
    for theta in np.linspace(0, pi/2, num_steps + 1):
        bins = [[[0, 0]], [[cos(theta), sin(theta)]]]
        model.update(bins)
    model.update([[[0, 0]], [[0, 0]]])
