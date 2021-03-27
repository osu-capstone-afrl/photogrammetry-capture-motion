from typing import List
import numpy as np


class Plane(object):
    """Container for defining a plane by a list of x-by-z equally spaced points"""
    def __init__(self, x_num, z_num, x_len, z_len):
        # type: (int, int, float, float) -> None
        """
        Initializes a plane abject as a list of [x,y,z] coordinates. Starts at the origin and builds
        in the positive x and positive z directions.

        @param x_num: Number of points to be placed in the x-direction / length
        @param z_num: Number of points to be placed in the z-direction / height
        @param x_len: Length of plane along the x-axis
        @param z_len: Length of plane along the z-axis
        """

        self._x_num = x_num
        self._z_num = z_num
        self._x_len = x_len
        self._z_len = z_len
        self._x_space = x_len / float(x_num - 1)
        self._z_space = z_len / float(z_num - 1)

        self._points = self._set_up_plane()

    def get_points(self):
        # type: () -> List[List[float]]
        """
        Getter function that returns a list of points [x, y, z] that describe the
        plane object.

        @return: List of lists formatted as [x, y, z]
        """
        return self._points

    def _set_up_plane(self):
        # type: () -> List[List[float]]
        """
        Internal function called by __init__ to place the points of the plane.
        Flips the order of every row for easier path tracing by the robot.

        @return: List of lists formatted as [x, y, z] that describe the plane
        """

        ret_list = []  # [[X0, Y0, Z0], [X1, Y1, Z1], ... ]  <- list of lists
        height = 0
        temp_list = [[0, 0, 0]]  # the temp list starts with the origin

        for i in range(self._z_num):
            for j in range(self._x_num - 1):
                temp_list.append([temp_list[-1][0] + self._x_space, temp_list[-1][1], temp_list[-1][2]])

            if i % 2 == 1:
                temp_list.reverse()

            ret_list += temp_list
            height += self._z_space
            temp_list = [] + [[ret_list[0][0], ret_list[0][1], ret_list[0][2] + height]]  # reset for next row

        return ret_list


class Ring(object):
    """Container for a ring defined by a list of equally spaced points around a circle"""
    def __init__(self, diameter, height, density=10):
        # type: (float, float, int) -> None
        """
        Creates a ring of point with the specified diameter and density. The ring is
        offset from the XY-plane by the height.

        Call Ring.get_points() to retrieve the points.

        @param diameter: Diameter of the ring
        @param height:   Distance from the XY-plane
        @param density:  How many points to place around the ring
        """
        theta = np.linspace(-np.pi, np.pi, density, endpoint=False)
        x = diameter * np.cos(theta)
        y = diameter * np.sin(theta)
        z = np.full_like(theta, height)

        self.ring = []
        for _x, _y, _z in zip(x, y, z):
            self.ring.append([_x, _y, _z])

    def get_points(self):
        # type: () ->  List[List[float]]
        """
        Getter for the list of points created upon calling __init__

        @return: List of [x, y, z] coordinates for the ring
        """
        return self.ring
