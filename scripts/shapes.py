import numpy as np


class Plane(object):
    """Container for defining a plane by a list of x-by-z equally spaced points"""
    def __init__(self, x_num, z_num, x_len, z_len):
        # type: (int, int, int, int) -> none
        """Initializes a plane abject as a list of [x,y,z] coordinates"""
        self._x_num = x_num
        self._z_num = z_num
        self._x_len = x_len
        self._z_len = z_len
        self._x_space = x_len / float(x_num - 1)
        self._z_space = z_len / float(z_num - 1)

        self.points = self._get_plane_points()

    def _get_plane_points(self):
        # type: () -> list([list[int]])
        """Returns all the points on the plane. Attempts to do this in a path-planning friendly way"""
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

    def rotate(self, axis, angle, rad=False):
        # type: (str, int, bool) -> none
        """Rotates points in a plane around a specific global axis by the specified angle"""
        if axis not in ['x', 'X', 'y', 'Y', 'z', 'Z']:
            print "[WARN] Invalid axis in Plane.rotate(), original list was not changed"
            return

        if not rad:
            angle *= np.pi / 180

        rot = np.identity(3)
        if axis in ['x', 'X']:
            rot[1][1] = np.cos(angle)
            rot[1][2] = np.sin(angle) * -1
            rot[2][1] = np.sin(angle)
            rot[2][2] = np.cos(angle)
        elif axis in ['y', 'Y']:
            rot[0][0] = np.cos(angle)
            rot[0][2] = np.sin(angle)
            rot[2][0] = np.sin(angle) * -1
            rot[2][2] = np.cos(angle)
        elif axis in ['z', 'Z']:
            rot[0][0] = np.cos(angle)
            rot[0][1] = np.sin(angle) * -1
            rot[1][0] = np.sin(angle)
            rot[1][1] = np.cos(angle)

        for p in self.points:
            p[:] = np.matmul(rot, p).tolist()

        return

    def translate(self, axis, dist):
        # type: (str, int) -> None
        """Translates points in a plane around a specified global axis by the specified distance"""
        if axis not in ['x', 'X', 'y', 'Y', 'z', 'Z', 'n', 'N']:
            print "[WARN] Invalid axis in Plane.translate(), original list was not changed"
            return

        for p in self.points:
            if axis in ['x', 'X']:
                p[0] += dist
            elif axis in ['y', 'Y']:
                p[1] += dist
            elif axis in ['z', 'Z']:
                p[2] += dist

        if axis in ['n', 'N']:
            n = self.get_surf_norm()
            for p in self.points:
                p[0] += n[0] * dist
                p[1] += n[1] * dist
                p[2] += n[2] * dist

        return

    def get_surf_norm(self):
        # type: () -> list[int]
        """Returns the normal vector to the plane, scaled to unit length"""
        # todo: check if the directions are consistent with what we expect
        a = np.subtract(self.points[0], self.points[1])
        b = np.subtract(self.points[0], self.points[-1])

        norm = np.cross(a, b)

        return (norm / (norm**2).sum()**0.5).tolist()



