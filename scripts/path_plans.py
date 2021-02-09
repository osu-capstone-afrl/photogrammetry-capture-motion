import numpy as np
from shapes import Plane
from math import radians
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def _calc_unit_vec_between(vec_a, vec_b): # type: (list[int], list[int]) -> list[int]
    """Finds the vector pointing from vec_a to vec_b and scales it to unit length"""
    vec = np.subtract(vec_b, vec_a)
    vec = vec / np.sqrt(np.sum(vec**2))
    return vec.tolist()


class DetectedObject(object):
    """A container to record the geometric information of a detected object"""
    # All variations of path plans will inherit from this...
    # The child plans will call their own functions to generate a list of positions
    # and orientations and will update the self.pose_and_orientation list.  At this point
    # there is no reason to call any member function or access any member variables other
    # than self.pose_and_orientation
    def __init__(self, size, center, orientation):
        # type: (list[int], list[int], int) -> None
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
        self._center = center
        self._x_loc = center[0]  # Double storing the data is not ideal. Refactor in the future
        self._y_loc = center[1]
        self._z_loc = center[2]
        self._orientation = orientation

        self.pose_and_orientation = []


class InclinedPlane(DetectedObject):
    """A container for the incline plane path plan"""
    def __init__(self, size, center, orientation, x_num=5, z_num=5, slope=0.5, offset=0.25):
        # type: (list[int], list[int], int, int, int, int, int) -> None
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
        # type () -> None
        """Sets up the 4 planes and store points as a List[dict{}] in member variable self.pose_and_orientation """
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

        for plane in [self._plane_length_1, self._plane_length_2, self._plane_width_1, self._plane_width_2]:
            for pose in plane.points:
                # Pose has all of the x,y,z positions.
                # Calculate the unit vector to the part's origin (parent function)
                unit = _calc_unit_vec_between(pose, self._center)

                # find the euler representation of that unit vector
                x_ang = np.arctan2(unit[1], unit[0])  # between x-axis and unit
                z_ang = np.arctan2(np.sqrt(unit[0]**2 + unit[1]**2), unit[2])  # between z-axis and unit
                angles = [x_ang, 0, z_ang]

                # Change that to quart
                # put in a dict
                msg = {"position": pose,
                       "quaternion": angles,
                       "unit": unit}  # unit is included for testing

                # print "Added the position: ", pose, " with unit ", unit, " and angles ", angles

                # push to the self list
                self.pose_and_orientation += [msg]

        # The pose_and_orientation element is now populated with a list of dictionaries

        return


''' TESTING AND VISUALIZATION BELOW '''


def main():
    """For testing code and visualizing the points"""
    dimensions = [0.14, 0.06, 0.04]
    center_loc = [1, 2, 3]
    twist_angle = 30

    blade = InclinedPlane(dimensions,
                          center_loc,
                          twist_angle,
                          offset=0.25)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    m = 'o'
    x = [center_loc[0]]
    y = [center_loc[1]]
    z = [center_loc[2]]
    u = [0]
    v = [0]
    w = [0]
    a = [0]
    b = [0]
    c = [0]

    for p in blade.pose_and_orientation:
        x += [p["position"][0]]
        y += [p["position"][1]]
        z += [p["position"][2]]

        u += [p["unit"][0]]
        v += [p["unit"][1]]
        w += [p["unit"][2]]

        a += [np.sin([p["quaternion"][2]])*np.cos([p["quaternion"][0]])]
        b += [np.sin([p["quaternion"][2]])*np.sin([p["quaternion"][0]])]
        c += [np.cos([p["quaternion"][2]])]

    ax.plot(x, y, z, color='k')
    ax.scatter(x, y, z)
    ax.quiver(x, y, z, u, v, w, length=0.1, normalize=True)
    ax.quiver(x, y, z, a, b, c, length=0.05, normalize=True, color='r')


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
