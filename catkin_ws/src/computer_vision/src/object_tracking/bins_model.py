import numpy as np
from math import pi


class BinsModel(object):
    yaw_dev = 0.17 # 10 degrees
    offset_length = 0.6 # 60 cm
    max_pos_error = 0.1 # 10 cm
    
    def __init__(yaw, position, num_bins):
        self.num_bins = num_bins

    @classmethod
    def generateConfigs(cls, locations, num_bins):
        if len(locations) == 0 or len(locations) > self.num_bins:
            return []
        yaws = [rectangleYaw(rect) for rect in locations]
        centers = [np.average(rect, axis=0) for rect in locations]
        
        # TODO: Need to be careful when calculating yaw avg and std about the limits of yaw. ugh
        if numpy.std(yaws) > yaw_dev:
            return []
        yaw = np.average(yaws)


        min_offset = 0
        max_offset = 0
        offset_dir = np.array([np.cos(yaw + pi/2), np.sin(yaw + pi/2)])
        errors = [[0, 0]]
        for i in range(0, len(locations) - 1):
            # check that the diff between successive locations mod spacing along yaw direction is small
            diff = locations[i + 1] - locations[i]
            proj = np.dot(diff, offset_dir)
            offset = np.round(proj / offset_length)
            error = diff - offset * offset_length * offset_dir
            if np.linalg.norm(error) > max_pos_error:
                return []

            # and average the diffs including a diff of zero for the first one to get offset from first bin
            errors.append(error)

            # and track the limiting bins
            if offset > max_offset:
                max_offset = offset
            if offset < min_offset:
                min_offset = offset

        if max_offset - min_offset > num_bins:
            return []

        center = locations[0] + np.average(error, axis=0)
        configurations = [BinsModel(yaw, center + i * offset_length * offset_dir, num_bins)
                for i in range(min_offset, max_offset + 1)]
            
    def update(locations):
        pass
