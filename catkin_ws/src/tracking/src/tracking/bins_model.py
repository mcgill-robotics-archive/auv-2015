import numpy as np


class LargePositionError(Exception):
    pass


class NoDirectionError(Exception):
    pass


class BinsModel(object):
    yaw_dev = 0.17  # 10 degrees
    spacing = 0.6  # 60 cm
    max_pos_error = 0.1  # 10 cm

    def __init__(self, direction, position, num_bins):
        '''
        Params:
            direction: Direction along which the bins are arrayed.
            position: Center of the first bin.
            num_bins: How many bins there are.
        '''
        self.direction = direction
        self.position = position
        self.num_bins = num_bins

    def update(self, locations):
        try:
            direction = self.calculate_bins_direction(locations)
        except NoDirectionError:
            return False
        if np.arccos(np.dot(self.direction, direction)) > self.yaw_dev:
            return False

        position, min_offset, max_offset = self.calculate_bins_locations(
            np.average(locations, axis=1), direction, self.spacing)

        if min_offset < 0 or max_offset >= self.num_bins:
            return False

        self.direction = direction
        self.position = position
        return True

    @classmethod
    def generate_configs(cls, locations, num_bins):
        if len(locations) == 0 or len(locations) > num_bins:
            return []

        try:
            direction = cls.calculate_bins_direction(locations)
        except NoDirectionError:
            return []

        try:
            center, min_offset, max_offset = cls.calculate_bin_locations(
                np.average(locations, axis=1), direction, cls.spacing)
        except LargePositionError:
            return []

        return [cls(direction, center + i * cls.spacing * direction, num_bins)
                for i in range(max_offset + 1 - num_bins, min_offset + 1)]

    @staticmethod
    def calculate_direction(points):
        try:
            eig_val, eig_vec = np.linalg.eig(np.cov(points))
            return eig_vec[np.argmax(eig_val, axis=0)]
        except np.linalg.LinAlgError:
            raise NoDirectionError

    @staticmethod
    def directed_distance_offset(p1, p2, direction, divisor):
        # Calculate the distance from p1 to p2 along the specified
        # direction, as an integer multiple of divisor plus a remainder
        diff = p2 - p1
        proj = np.dot(diff, direction)
        quotient = np.round(proj / divisor)
        remainder = diff - quotient * divisor * direction
        return quotient, remainder

    @classmethod
    def calculate_bins_direction(cls, locations):
        # If we have multiple objects we look at the objects to find which
        # direction they are arranged along. If we only have one rectangle then
        # we choose the direction perpendicular to its long axis.
        if len(locations) == 1:
            long_dir = cls.calculate_direction(locations[0])
            return [long_dir[1], -long_dir[0]]
        else:
            centers = np.average(locations, axis=1)
            return cls.calculate_direction(centers)

    @classmethod
    def calculate_bins_locations(cls, centers, direction, spacing):
        min_offset = 0
        max_offset = 0
        errors = [[0, 0]]
        for i in range(0, len(centers) - 1):
            # check that the diff between successive locations mod spacing
            # along yaw direction is small.
            offset, error = cls.directed_distance_offset(
                centers[i], centers[i+1], direction, spacing)
            if np.linalg.norm(error) > cls.max_pos_error:
                raise LargePositionError

            # and average the diffs including a diff of zero for the first one
            # to get offset from first bin.
            errors.append(error)

            # and track the limiting bins
            if offset > max_offset:
                max_offset = offset
            if offset < min_offset:
                min_offset = offset

        center = centers[0] + np.average(error, axis=0)
        return center, min_offset, max_offset
