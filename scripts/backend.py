import numpy as np
from geo_obj import Plane
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class DetectedObject(object):
    """A container to record the geometric information of a detected object"""
    # todo: add type assistance
    def __init__(self, size, center, orientation, x_num=5, z_num=5, slope=0.5):
        # Assumes that the size and center arrays are ordered from largest to smallest
        # Assumes that the z dimension is the smallest
        # Assumes that theta is the angle between the longest axis and the base X axis
        #
        # i.e. this works best if the part is laying as flat as possible
        # and the longest dimension is on the robot's x axis

        self._x_len = size[0]
        self._y_len = size[1]
        self._z_len = size[2]
        self._x_loc = center[0]
        self._y_loc = center[1]
        self._z_loc = center[2]
        self._orientation = orientation
        self._x_num = z_num
        self._z_num = z_num
        if 0 <= slope <= 1:
            self._slope = slope
        elif slope > 1:
            self._slope = 1
        elif slope < 0:
            self._slope =0

        # two planes for along the length
        self._plane_length_1 = Plane(x_num, z_num, size[0], size[2])
        self._plane_length_2 = Plane(x_num, z_num, size[0], size[2])
        # two planes for along the width
        self._plane_width_1 = Plane(x_num, z_num, size[1], size[2])
        self._plane_width_2 = Plane(x_num, z_num, size[1], size[2])

        self._set_up_planes()

        return

    def _set_up_planes(self):

        ## length planes
        # translate length planes back by length/2
        self._plane_length_1.translate('x', self._x_len / -2)
        self._plane_length_2.translate('x', self._x_len / -2)
        # rotate length planes on x by  a factor of theta max
        theta = self._slope * np.arctan(self._y_len / self._z_len)
        self._plane_length_1.rotate('x', theta, rad=True)
        self._plane_length_2.rotate('x', theta * -1, rad=True)
        # push on y length planes out by width
        self._plane_length_1.translate('y', self._y_len)
        self._plane_length_2.translate('y', self._y_len * -1)

        ## width planes
        self._plane_width_1.rotate('z', 90)
        self._plane_width_2.rotate('z', 90)
        self._plane_width_1.translate('y', self._y_len / -2)
        self._plane_width_2.translate('y', self._y_len / -2)
        # rotate around y by a factor of phi max
        phi = self._slope * np.arctan(self._x_len / self._z_len)
        self._plane_width_1.rotate('y', phi * -1, rad=True)
        self._plane_width_2.rotate('y', phi, rad=True)
        # push on x by length
        self._plane_width_1.translate('x', self._x_len)
        self._plane_width_2.translate('x', self._x_len * -1)

        ## Planes are now defined relative to the bounding box

        # I think these work properly...
        for plane in [self._plane_length_1, self._plane_length_2, self._plane_width_1, self._plane_width_2]:
            # p = plane.points
            plane.rotate('z', self._orientation)
            plane.translate('x', self._x_loc)
            plane.translate('y', self._y_loc)
            plane.translate('z', self._z_loc)

        ## Planes are now defined relative to the robot's origin

        return

    def get_positions(self):
        post_and_orient = []

        for plane in [self._plane_length_1, self._plane_length_2, self._plane_width_1, self._plane_width_2]:
            for point in plane.points:
                orientation = self._get_orientations(point)
                post_and_orient += [point + orientation]

        return post_and_orient

    # todo: find the orientation to use
    def _get_orientations(self, vec):
        return [0, 0, 0]


# todo: move plane implementation from detect obj to this (smh)
class InclinedPlane(DetectedObject):
    """A container for the incline plane path plan"""

    def __init__(self, center, x_len, y_len, z_len, theta):
        super(InclinedPlane, self).__init__(center, x_len, y_len, z_len, theta)


''' TESTING AND VISUALIZATION BELOW '''


def main():
    blade = DetectedObject([0.14, 0.06, 0.04],
                           [0., 0., 0.],
                           0)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    m = 'o'
    for p in blade.get_positions():
        ax.scatter(p[0], p[1], p[2], marker=m)

    ax.set_xlabel('X Label')
    # ax.set_xlim([0, 0.4])
    ax.set_ylabel('Y Label')
    # ax.set_ylim([0, 0.4])
    ax.set_zlabel('Z Label')
    # ax.set_zlim([0, 0.1])

    plt.show()


if __name__ == "__main__":
    main()
