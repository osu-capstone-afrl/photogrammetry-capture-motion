import numpy as np
from geo_obj import Plane
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class DetectedObject(object):
    """A container to record the geometric information of a detected object"""
    def __init__(self, center, x_len, y_len, z_len, theta):
        self._center = center
        self._x_len = x_len
        self._y_len = y_len
        self._z_len = z_len
        self._theta = theta


class InclinedPlane(DetectedObject):
    """A container for the incline plane path plan"""

    def __init__(self, center, x_len, y_len, z_len, theta):
        super(InclinedPlane, self).__init__(center, x_len, y_len, z_len, theta)


''' TESTING AND VISUALIZATION '''

test_plane = Plane(6, 6, 5, 5)
# test_plane.translate('x',3)  # PASS
# test_plane.translate('z',3)  # PASS
# test_plane.translate('v',3)  # PASS
# test_plane.rotate('z', 45)  # PASS
# test_plane.rotate('y', 45)  # PASS
# test_plane.rotate('q', 45)  # PASS

test_plane_2 = Plane(8, 6, 5, 3)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

m = 'o'
for p in test_plane.points:
    xs = p[0]
    ys = p[1]
    zs = p[2]
    ax.scatter(xs, ys, zs, marker=m)

n = '^'
for p in test_plane_2.points:
    xs = p[0]
    ys = p[1]
    zs = p[2]
    ax.scatter(xs, ys, zs, marker=n)

ax.set_xlabel('X Label')
ax.set_xlim([-5, 5])
ax.set_ylabel('Y Label')
ax.set_ylim([-5, 5])
ax.set_zlabel('Z Label')
ax.set_zlim([-5, 5])


plt.show()

