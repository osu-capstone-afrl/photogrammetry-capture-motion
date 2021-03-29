from visualizations import plot_path_transforms
from transformations import Transformations
from shapes import Plane
from shapes import Ring
from typing import List
from typing import Tuple
from typing import Dict
import numpy as np


class DetectedObject(object):
    """A container for the geometric information of a detected object template for all path plans"""
    def __init__(self, size, locator, orientation):
        # type: (List[float], List[float], np.ndarray) -> None
        """
        **Usage**: Parent class for all path plans. Contains pose and geometry of the part common to
        all path plans, _size, _orientation, and _locator. Instantiates a transformations class to
        handle any transformations needed by the path plan.

        usage: Parent and template for all path plans

        @param size:        list[x,y,z] dimensions of presented part
        @param locator:     list[x,y,z] center of presented part in world frame coordinates
        @param orientation: 3x3 rotation matrix describing the part's orientation
        """

        # explicitly set the robot's origin at the center
        self._global_origin = np.array([0., 0., 0.])

        self.tf = Transformations()
        self._locator_tf = self.tf.get_transformation(orientation, locator)

        self._size = size
        self._locator = locator
        self._orientation = orientation

        ''' OVERWRITE THESE VARIABLES IN YOUR PATH PLAN '''
        # Holds path as a list of transformations relative to _global_origin (0,0,0)
        self.path_as_transforms = self._setup_path()

        # Holds path to Robot Poses (outputs a list of vectors x,y,z,qx,qy,qz,qw)
        self.path_as_poses = self.tf.convert_transformations_to_poses(self.path_as_transforms)

        # Holds path as a list of dictionaries holding ROS pose messages
        # see http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html
        # [{'Point': [x, y, z], 'Quaternion': [qx, qy, qz, qw]}]
        self.path_as_messages = self._convert_poses_to_messages(self.path_as_poses)

    # Can access the variables directly or with type help via these getter functions
    def get_path_as_transforms(self):
        # type: () -> List[np.ndarray]
        """
        Explicitly gets the path as a list of homogenous transforms
        @return: path as a list of homogenous transforms
        """
        return self.path_as_transforms

    def get_path_as_poses(self):
        # type: () -> List[List[float]]
        """
        Explicitly gets the path as a list of poses
        @return: path as a list of poses [x, y, z, qw, qy, qz, qw]
        """
        return self.path_as_poses

    def get_path_as_messages(self):
        # type: () -> List[Dict]
        """
        Explicitly gets the path as a list of ROS pose messages
        @return: path as a list of pose message {'Point': [x, y, z], 'Quaternion': [qx, qy, qz, qw]}
        """
        return self.path_as_messages

    def _setup_path(self):
        # type: () -> List[np.ndarray]
        """
        Creates a list of path transformations that position the camera around
        the part and orient the Z-axis towards the part's center location with the
        Y-axis parallel to the XY plane and X-axis pointing up.

        Usage: automatically called by __init__

        @return: The path as a list of transformation matrices
        """
        ''' OVERWRITE THIS FUNCTION IN YOUR PATH PLAN '''
        path_as_transforms = []
        return path_as_transforms

    @staticmethod
    def _convert_poses_to_messages(path_poses):
        # type: (List[List[float]]) -> List[Dict]
        """ Takes a path as a list of poses and formats them into a
        dictionary per: http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html

        Each element in the returned list looks like:
        msg = {'Position': [x, y, z], 'Orientation': [x, y, z, w]}

        @param path_poses: List of poses as [x, y, z, qx, qy, qz, qw]
        @return: list of poses formatted as a dictionary
        """
        import geometry_msgs.msg

        path_messages = []
        #pose_template = geometry_msgs.msg.Pose()

        for pose in path_poses:
            pose_goal               = geometry_msgs.msg.Pose()
            pose_goal.position.x    = pose[0]
            pose_goal.position.y    = pose[1]
            pose_goal.position.z    = pose[2]
            pose_goal.orientation.x = pose[3]
            pose_goal.orientation.y = pose[4]
            pose_goal.orientation.z = pose[5]
            pose_goal.orientation.w = pose[6]

            path_messages.append(pose_goal)

        return path_messages

    def _get_z_axis_oriented_rotation(self, camera_position):
        # type: (List[float]) -> np.ndarray
        """
        Generate a rotation matrix to orient the resultant Z-axis along a vector
        between a input camera position (local_point) and the central part location.
        Defined such that tool frame's Y-axis is constrained to a plane parallel with floor.

        @param camera_position: local_point: (list) Input Camera Position in cartesian coordinates [X,Y,Z]
        @return rot_matrix: 3x3 Rotation Matrix which orients the Z-axis towards the global origin
        """

        # World Frame (ignoring translation)
        vector_z = np.subtract(self._global_origin, camera_position)

        # Create two test vectors
        vector_y_1 = [-vector_z[1], vector_z[0], 0]
        vector_y_2 = [vector_z[1], -vector_z[0], 0]

        # Find third vector (order matters, must multiply <Y> cross <Z> to follow right-hand-rule)
        vector_x_1 = np.cross(vector_y_1, vector_z)
        vector_x_2 = np.cross(vector_y_2, vector_z)

        # Check Signs to find proper quadrant
        if vector_z[0] == 0 and vector_z[1] == 0:
            # Special Case. Vector Z, collinear with Z-Axis
            vector_x = np.array([1, 0, 0])
            vector_y = np.array([0, 1, 1])
        elif vector_x_1[2] > 0:
            vector_x = vector_x_1
            vector_y = vector_y_1
        elif vector_x_2[2] > 0:
            vector_x = vector_x_2
            vector_y = vector_y_2

        # Normalize
        vector_x = vector_x / np.linalg.norm(vector_x)
        vector_y = vector_y / np.linalg.norm(vector_y)
        vector_z = vector_z / np.linalg.norm(vector_z)

        # Rotation Matrix
        # https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Formalism_alternatives
        rot_matrix = np.stack((vector_x, vector_y, vector_z), axis=1)

        return rot_matrix


class NewInclinedPlane(DetectedObject):
    """A container for the incline plane path plan"""
    def __init__(self, size, locator, orientation, count=(5, 5), clearance=0.25, plane_scale=(2, 2),\
                 slope=0.5, offset=0.25):
        # type: (List[float], List[float], np.ndarray, Tuple[int,int], float, Tuple[float,float], float, float) -> None
        """Initialization function

        @param size:        list[x,y,z] dimensions of presented part
        @param locator:     center of presented part in world frame coordinates
        @param orientation: rotation of presented part in world frame coordinates
        @param count:       Tuple (X,Z) with the count of points along each direction of the plane
        @param clearance:   Distance between each plane and the part before the planes are inclined
        @param plane_scale: Tuple (X,Z) describing the how the planes dimensions are scaled relative to the part's
        @param slope:       Multiplier from 0 (no tilt) to 1 (maximum tilt) to adjust the incline of each plane
        @param offset:      Final adjustment which moves the position backwards along the camera's Z-axis
        """

        (self._x_count, self._z_count) = count
        self._clearance = clearance
        (self._x_scale, self._z_scale) = plane_scale

        if 1 < slope or slope < 0:
            slope = 0.5
        self._slope = slope
        self._offset = offset

        super(NewInclinedPlane, self).__init__(size, locator, orientation)

        ''' OVERWRITE THESE VARIABLES IN YOUR PATH PLAN '''
        # Holds path as a list of transformations relative to _global_origin (0,0,0)
        self.path_as_transforms = self._setup_path()

        # Holds path to Robot Poses (outputs a list of vectors x,y,z,qx,qy,qz,qw)
        self.path_as_poses = self.tf.convert_transformations_to_poses(self.path_as_transforms)

        # Holds path as a list of dictionaries holding ROS pose messages
        # see http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html
        # [{'Point': [x, y, z], 'Quaternion': [qx, qy, qz, qw]}]
        self.path_as_messages = self._convert_poses_to_messages(self.path_as_poses)

    def _setup_path(self):
        # type: () -> List[np.ndarray]
        """
        Creates a list of path transformations that position the camera around
        the part and orient the Z-axis towards the part's center location with the
        Y-axis parallel to the XY plane and X-axis pointing up.

        Usage: automatically called by __init__
        """

        x_len = self._x_scale*self._size[0]
        y_len = self._x_scale*self._size[1]
        z_len = self._z_scale*self._size[2]

        plane_xz1 = Plane(self._x_count, self._z_count, x_len, z_len)
        plane_xz2 = Plane(self._x_count, self._z_count, x_len, z_len)
        plane_yz1 = Plane(self._x_count, self._z_count, y_len, z_len)
        plane_yz2 = Plane(self._x_count, self._z_count, y_len, z_len)

        plane_xz1_offset = np.array([-0.5*x_len,     0.5*self._size[1] + self._clearance,  0])
        plane_xz2_offset = np.array([-0.5*x_len, -1*(0.5*self._size[1] + self._clearance), 0])
        plane_yz1_offset = np.array([    0.5*self._size[0] + self._clearance,  -0.5*y_len, 0])
        plane_yz2_offset = np.array([-1*(0.5*self._size[0] + self._clearance), -0.5*y_len, 0])

        # Find how far we can tilt each plane, scale that based on self._slope
        h = 0.5*self._size[2]
        theta = self._slope*np.arctan(h / self._clearance)

        # each plane's rotation
        rot_xz1 = self.tf.create_rotation_matrix([theta], 'x')
        rot_xz2 = self.tf.create_rotation_matrix([-1*theta], 'x')
        rot_yz1 = self.tf.create_rotation_matrix([0.5*np.pi, -1*theta], 'zy')
        rot_yz2 = self.tf.create_rotation_matrix([0.5*np.pi, theta], 'zy')

        # combine into a transformation matrix to place planes around global origin
        tf_xz1 = self.tf.get_transformation(rot_xz1, plane_xz1_offset)
        tf_xz2 = self.tf.get_transformation(rot_xz2, plane_xz2_offset)
        tf_yz1 = self.tf.get_transformation(rot_yz1, plane_yz1_offset)
        tf_yz2 = self.tf.get_transformation(rot_yz2, plane_yz2_offset)

        no_rotation = np.identity(3)
        tf_final_offset = self.tf.get_transformation(no_rotation, [0, 0, -1*self._offset])

        # Create rotation transformations
        path_as_transforms = []
        planes = [plane_xz1, plane_yz1, plane_xz2, plane_yz2]
        for i, plane in enumerate(planes):
            for p in plane.get_points():
                # planes are just List[x,y,z], to move them around we make them info a vector
                # [x, y, z, 1]' and apply homogenous transforms
                p.append(1)
                point = np.asarray(p).reshape(4)

                # place around origin
                if i == 0:  # xz1
                    point_around_origin = np.matmul(tf_xz1, point)
                elif i == 1:  # yz1
                    point_around_origin = np.matmul(tf_yz1, point)
                elif i == 2:  # xz2
                    point_around_origin = np.matmul(tf_xz2, point)
                else:  # 'yz2'
                    point_around_origin = np.matmul(tf_yz2, point)

                # Now that the points are spaced around the origin we can get their orientation
                # to describe them as a full homogeneous matrix
                xyz = point_around_origin[:-1]
                point_rot = self._get_z_axis_oriented_rotation(xyz)
                tf_around_origin = self.tf.get_transformation(point_rot, xyz)

                # place around part and apply offset
                tf_around_part = np.matmul(self._locator_tf, tf_around_origin)

                # post to move relative to each frame instead of the world frame
                tf_with_offset = np.matmul(tf_around_part, tf_final_offset)

                path_as_transforms.append(tf_with_offset)

        return path_as_transforms


class SteppedRings(DetectedObject):
    """A container for the stepped rings shape"""
    def __init__(self, size, locator, orientation, scale=1.1, offset=0.2, level_count=5, density=10):
        # type: (List[float], List[float], np.ndarray, float, float, int, int) -> None
        """
        Path Plan generator for the Stepped Rings Shape. Stacked rings extending from base
        to 110% of input part height. Sets SteppedRings's internal variables and calls
        self._setup_rings() to generate the poses for the path plan.


        Usage: Create a path plan object and this will automatically populate the
        self.pose_list variable.

        @param size:        list[x,y,z] dimensions of presented part
        @param locator:     center of presented part in world frame coordinates
        @param orientation: rotation of presented part in world frame coordinates
        @param scale:       multiplier to the minimum diameter
        @param offset:      constant distance added to the minimum diameter (after scaling)
        @param level_count: number of stepped levels
        @param density:     number of points on a single level. Evenly distributed about the ring
        """

        self._min_diameter = np.hypot(size[0], size[1])*scale + offset
        self._density = density

        # Select all level heights except the fist which is at z=0
        self._level_heights = np.linspace(0, size[2]*1.10, level_count+1)[1:]

        super(SteppedRings, self).__init__(size, locator, orientation)

        # Holds path as a list of transformations relative to _global_origin (0,0,0)
        self.path_as_transforms = self._setup_path()

        # Holds path to Robot Poses (outputs a list of vectors x,y,z,qx,qy,qz,qw)
        self.path_as_poses = self.tf.convert_transformations_to_poses(self.path_as_transforms)

        # Holds path as a list of dictionaries holding ROS pose messages
        # see http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html
        # [{'Point': [x, y, z], 'Quaternion': [x, y, z, w]}]
        self.path_as_messages = self._convert_poses_to_messages(self.path_as_poses)

    def _setup_path(self):
        # type: () -> List[np.ndarray]
        """
        Creates a list of path transformations that position the camera around
        the part and orient the Z-axis towards the part's center location with the
        Y-axis parallel to the XY plane and X-axis pointing up.

        usage: automatically called by __init__

        @return: path as a list of transformations
        """
        path_as_transforms = []
        for level in self._level_heights:
            ring = Ring(self._min_diameter, level, self._density)

            for point in ring.get_points():
                rot_matrix = self._get_z_axis_oriented_rotation(point)
                tf_around_origin = self.tf.get_transformation(rot_matrix, point)

                tf_around_part = np.matmul(self._locator_tf, tf_around_origin)
                path_as_transforms.append(tf_around_part)

        return path_as_transforms


def main():
    # For testing code and visualizing the points
    print("--- DEMO CODE: Must enable method (InclinedPlane, SteppedRings ----\n")

    # Collision Box Size, Location, Angle
    tf = Transformations
    dimensions = [0.5, 0.5, 0.5]
    center_loc = [0.1, 0.1, 0.1]
    orientation = tf.create_rotation_matrix()

    # Stepped Rings Visualization Demo
    if False:
        print("......Method: SteppedRings ")
        demo_rings = SteppedRings(dimensions, center_loc, orientation)
        plot_path_transforms(demo_rings.get_path_as_transforms())

    if True:
        print("......Method: NewPlanes ")
        demo = NewInclinedPlane(dimensions, center_loc, orientation, \
                                count=(5,5), slope=0.5, clearance=1, offset=3)
        plot_path_transforms(demo.get_path_as_transforms())


if __name__ == "__main__":
    main()
