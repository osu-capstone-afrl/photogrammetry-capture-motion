import numpy as np
from geo_obj import Plane
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


# todo: add type assistance to all functions

class DetectedObject(object):
    """A container to record the geometric information of a detected object"""
    def __init__(self, size, center, orientation):
        """Initialization function"""
        # Assumes that the size and center arrays are ordered from largest to smallest
        # Assumes that the z dimension is the smallest
        # Assumes that theta is the angle between the longest axis and the base X axis
        #
        # i.e. this works best if the part is laying as flat as possible
        # and the longest dimension is closely aligned to the robot's x axis

        self._x_len = size[0]
        self._y_len = size[1]
        self._z_len = size[2]
        self._x_loc = center[0]
        self._y_loc = center[1]
        self._z_loc = center[2]
        self._orientation = orientation


class InclinedPlane(DetectedObject):
    """A container for the incline plane path plan"""
    def __init__(self, size, center, orientation, x_num=5, z_num=5, slope=0.5, offset=0.25):
        """Initialization function"""
        super(InclinedPlane, self).__init__(size, center, orientation)
        self._x_num = z_num
        self._z_num = z_num
        if 0 <= slope <= 1:
            self._slope = slope
        elif slope > 1:
            self._slope = 1
        elif slope < 0:
            self._slope =0
        self._offset = offset

        # two planes for along the length
        self._plane_length_1 = Plane(x_num, z_num, size[0], size[2])
        self._plane_length_2 = Plane(x_num, z_num, size[0], size[2])
        # two planes for along the width
        self._plane_width_1 = Plane(x_num, z_num, size[1], size[2])
        self._plane_width_2 = Plane(x_num, z_num, size[1], size[2])

        self._set_up_planes()

        return

    def _set_up_planes(self):
        """Internal function to set the four camera position planes"""
        # Length planes
        # Translate length planes back by length/2
        self._plane_length_1.translate('x', self._x_len / -2)
        self._plane_length_2.translate('x', self._x_len / -2)
        # Rotate length planes on x by  a factor of theta max
        theta = self._slope * np.arctan(self._y_len / self._z_len)
        self._plane_length_1.rotate('x', theta, rad=True)
        self._plane_length_2.rotate('x', theta * -1, rad=True)
        # Push on y length planes out by width
        self._plane_length_1.translate('y', self._y_len)
        self._plane_length_2.translate('y', self._y_len * -1)
        # Push on normal by the offset amount
        self._plane_length_1.translate('n', self._offset * -1)
        self._plane_length_2.translate('n', self._offset)

        # Width planes
        self._plane_width_1.rotate('z', 90)
        self._plane_width_2.rotate('z', 90)
        self._plane_width_1.translate('y', self._y_len / -2)
        self._plane_width_2.translate('y', self._y_len / -2)
        # Rotate around y by a factor of phi max
        phi = self._slope * np.arctan(self._x_len / self._z_len)
        self._plane_width_1.rotate('y', phi * -1, rad=True)
        self._plane_width_2.rotate('y', phi, rad=True)
        # Push on x by length
        self._plane_width_1.translate('x', self._x_len)
        self._plane_width_2.translate('x', self._x_len * -1)
        # Push on normal by the offset amount
        self._plane_width_1.translate('n', self._offset)
        self._plane_width_2.translate('n', self._offset * -1)

        # All planes are now defined relative to the bounding box

        for plane in [self._plane_length_1, self._plane_length_2, self._plane_width_1, self._plane_width_2]:
            # p = plane.points
            plane.rotate('z', self._orientation)
            plane.translate('x', self._x_loc)
            plane.translate('y', self._y_loc)
            plane.translate('z', self._z_loc)
        # All planes are now defined relative to the robot's origin
        return

    def get_positions(self):
        """Returns a list dictionaries of all positions and orientations"""
        post_and_orient = []
        msg = {'position': [0, 0, 0],
               'quaternion': [0, 0, 0, 0]}

        for plane in [self._plane_length_1, self._plane_length_2, self._plane_width_1, self._plane_width_2]:
            for point in plane.points:
                msg["position"] = point
                msg["quaternion"] = self._get_orientations(point) #TODO== Currently returns Hard Coded Value
                post_and_orient += [msg]

        return post_and_orient

    # todo: calculate orientatin from provided vector. (adam- maybe need reference frame input as well?)
    def _get_orientations(self, vec):
        """Returns a vector with the Quaternion orientation for a position"""
        from math import radians
        from tf.transformations import quaternion_from_euler

        #TODO: hardcoded to orient straight down.
        #orientation_euler = []
        #

        return list(quaternion_from_euler(0, radians(90), 0)) #Output list of quants in order x.y.z.w

''' TESTING AND VISUALIZATION BELOW '''


def main():
    """For testing code and visualizing the points"""
    blade = InclinedPlane([0.14, 0.06, 0.04],
                          [0., 0., 0.],
                          0,
                          offset=0.25)

    blade2 = InclinedPlane([0.14, 0.06, 0.04],
                           [0., 0., 0.],
                           0,
                           offset=0)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    m = 'o'
    x = []
    y = []
    z = []
    for p in blade.get_positions():
        x += [p["Position"][0]]
        y += [p["Position"][1]]
        z += [p["Position"][2]]

    # todo:  update for better visualization
    ax.plot(x, y, z, color='k')
    ax.scatter(x, y, z)

    # Limits
    ax.set_xlabel('X Label')
    #ax.set_xlim([-0.3, 0.3])
    ax.set_ylabel('Y Label')
    #ax.set_ylim([-0.3, 0.3])
    ax.set_zlabel('Z Label')
    #ax.set_zlim([-0.3, 0.3])

    plt.show()


if __name__ == "__main__":
    main()
