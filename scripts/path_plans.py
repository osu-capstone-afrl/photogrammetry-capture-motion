import numpy as np
from shapes import Plane
from math import radians
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def _calc_unit_vec_between(vec_a, vec_b): # type: (list[float], list[float]) -> list[float]
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
    def _point_method(self, size, center, rotation):
        # type: (list[float], list[float], float) -> None
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
        self._orientation = rotation

        self.pose_and_orientation = []

    def _tf_method(self, center, rotation):

        from transformations import transformations
        self.tf = transformations()

        self._locator = self.tf.generateTransMatrix(rotation,center)


class InclinedPlane(DetectedObject):
    """A container for the incline plane path plan"""
    def __init__(self, size, center, rotation, x_num=5, z_num=5, slope=0.5, offset=0.25):
        # type: (list[float], list[float], float, int, int, float, float) -> None
        """Initialization function

        :param size:      list[x,y,z] dimensions of presented part
        :param center:    center of presented part in world frame coordinates
        :param rotation:  rotation of presented part in world frame coordinates
        :param x_num:     number of stepped levels
        :param z_num:     number of points on a single level. Evenly distributed about the ring
        :param slope:     factor [0,1] to adjust the tilt of each plane, 0 implies no tilt
        :param offset:    Adjusts final position away from part along the each planes surface normal
        """
        super(InclinedPlane, self)._point_method(size, center, rotation)
        self._x_num = z_num
        self._z_num = z_num
        if 0 <= slope <= 1:
            self._slope = slope
        elif slope > 1:
            self._slope = 1
        elif slope < 0:
            self._slope = 0
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
                       "euler": angles,
                       "unit": unit}  # unit is included for testing

                # print "Added the position: ", pose, " with unit ", unit, " and angles ", angles

                # push to the self list
                self.pose_and_orientation += [msg]

        # The pose_and_orientation element is now populated with a 3-element dictionary, each element containing a list.

        return



class SteppedRings(DetectedObject):
    """ Path Plan generator for the Stepped Rings Shape """
    def __init__(self, size, center, rotation, level_count=5, density=10):
        """
        Call internal variables _path_tf or _path_pose

        :param size:        list[x,y,z] dimensions of presented part
        :param center:      center of presented part in world frame coordinates
        :param rotation:    rotation of presented part in world frame coordinates
        :param level_count: number of stepped levels
        :param density:     number of points on a single level. Evenly distributed about the ring
        """

        # Setup Root Structure
        # rotation frame hard coded to match fixed source frame
        rotation = np.identity(3)
        super(SteppedRings, self)._tf_method(center,rotation)

        # Set smallest ring (w/o buffer)(diagonal length of rectanglular base)
        self._min_diameter = np.hypot( size[0], size[1] )

        # Set number and final height of levels (distribute n levels across height)
        self._levels = np.linspace(0, size[2], level_count+1)

        # Passthru variables
        self._density = density

        # Create rings
        self._setup_rings()

        return


    def _setup_rings(self):

        # Generation
        path_result = []
        for lvl in self._levels[0:]:

            ## DEBUG
            print("================================================== LEVEL #: " + str(lvl))

            # Create Ring
            pheta = np.linspace(-np.pi, np.pi, self._density, endpoint=False)
            xx = (self._min_diameter * np.cos(pheta))
            yy = (self._min_diameter * np.sin(pheta))
            zz = (np.full_like(pheta, lvl))

            # Create Transforms
            for x,y,z in zip(xx,yy,zz):

                # Find Orientation. ie rotation matrix
                rot_matrix_vectors = self._find_rot_matrix([x,y,z])
                rot_matrix = np.matmul(np.identity(3), rot_matrix_vectors)
                #rot_matrix = np.identity(3)

                # Generate transforms list
                path_result.append( self.tf.generateTransMatrix(rot_matrix,[x,y,z]) )

        # Shift path body to be framed in context of Global Fixed Frame (0,0,0)
        self._path_tf = self.tf.convertPath2FixedFrame(path_result, self._locator)

        # Convert path to Robot Poses (outputs a list of vectors x,y,z,qx,qy,qz,qw)
        self._path_pose = self.tf.convertPath2RobotPose(self._path_tf)
        
        return


    def _find_rot_matrix(self, local_point):
        """
            Calculate and return rotation matrix to orient the resultant Z-axis along a vector
            between a input camera position (local_point) and the central part location.

            :param local_point: (list) Input Camera Position in cartesian coordinates [X,Y,Z]
            :return rot_matrix: Rotation Matrix as 3x3 Numpy Matrix 
        """


        # DEBUG
        print("-------- NEW ANGLE-----------------------")

        # World Frame (ignoring translation)
        vect_og = np.subtract([0,0,0], local_point)
        uvect_og = vect_og / np.linalg.norm(vect_og)

        print('Vector',np.around(vect_og,2))
        print('Unit Vector',np.around(uvect_og,2))

        # New Tool Frame
        uvect_z = np.array([0,0,1])


        ###################
        ## Find Rot Matrix Conversion from World to Tool frame

        # Find ROT_MATRIX necessary to rotation vector 'a' onto 'b'
        a = uvect_z     # unit z vector
        b = uvect_og    # point to center


        ###################
        ## METHOD 1
        ## Axis-Angle Rotation Method from Textbook. Ref. "Modern Robotics" Section 3.2 Pg 72 OR Eqn 3.52 
        # http://hades.mech.northwestern.edu/images/2/25/MR-v2.pdf#equation.3.52

        # Trash. Do Not use.
        if False:
            w = np.cross(a,b)
            c = np.dot(a,b)                      # cos(pheta)
            s = np.linalg.norm(np.cross(a,b),2)  # sin(pheta)

            R = np.array([[ c+np.square(w[0]) * (1-c),      w[0]*w[1]*(1-c)-w[2]*s,         w[0]*w[2]*(1-c)+w[1]*s   ],
                          [ w[0]*w[1]*(1-c)+w[2]*s,         c+np.square(w[1])*(1-c),        w[1]*w[2]*(1-c)-w[0]*s   ],
                          [ w[0]*w[2]*(1-c)-w[1]*s,         w[1]*w[2]*(1-c)+w[0]*s,         c+np.square(w[2])*(1-c)  ]] )        
            rot_matrix = R


        ###################
        ## METHOD 2
        # Based on https://math.stackexchange.com/a/897677
        # Solving Equation.. rot_matrix = F^-1 * G * F
        # All numpy.linalg.norm()'s are set to use L-2 norm. NOT Frobenius norm.
        if False:
            G = np.array( [[ np.dot(a,b),                     -np.linalg.norm(np.cross(a,b),2), 0],
                          [ np.linalg.norm(np.cross(a,b),2),   np.dot(a,b),                     0],
                          [ 0,                                 0,                               1]] )

            F = np.linalg.inv( np.stack( [  a, (b-np.dot(a,b)*a)/np.linalg.norm(b-np.dot(a,b)*a,2), np.cross(b,a)]) )

            rot_matrix = np.matmul( np.matmul( F, G), np.linalg.inv(F))


        ###################
        ## METHOD 3
        # Based on https://math.stackexchange.com/a/476311
        if True:
            v = np.cross(a,b)

            c = np.dot(a,b)                      # cos(pheta)
            s = np.linalg.norm(np.cross(a,b),2)  # sin(pheta)

            #DEBUG: Output cos & sin values to determine if quantrant issues.
            print('cos:',np.around(c,3))
            print('sin:',np.around(s,3))

            v_x = np.array( [[ 0,      -v[2],   v[1] ],
                             [ v[2],    0,     -v[0] ],
                             [-v[1],    v[0],   0    ]] )
            rot_matrix = np.identity(3) + v_x + np.dot(v_x,v_x) * (1/(1+c))

            print(np.around(rot_matrix,3))


        ################ Method 4. Manual Rotation ####################
        if False:
            #TODO: Does this projection make sense??
            vect_og2 = np.array( [ uvect_og[0], uvect_og[1], 0 ] )

            pheta_prime_z = np.arctan2(vect_og[1],vect_og[0])
            phi_prime_y = np.arccos( (np.dot(vect_og,vect_og2) ) / (np.linalg.norm(vect_og,2)*np.linalg.norm(vect_og2,2))  )


            ## Generate Rotation Matrices
            s = np.sin(pheta_prime_z)
            c = np.cos(pheta_prime_z)

            rot_z = np.matrix([ [ c, -s, 0],
                                [ s,  c, 0],
                                [ 0,  0, 1] ])

            s = np.sin(phi_prime_y)
            c = np.cos(phi_prime_y)

            rot_y = np.matrix([ [ c, 0, s],
                                [ 0, 1, 0],
                                [-s, 0, c] ])

            # Rotation Matrix (Rotation w.r.t fixed frame)(pre-multiply)
            rot_matrix = np.matmul(rot_z,rot_y)


        ##############################################################

        # DEBUG
        if False:
            print("Locator: ", self._locator[:-1,3])
            print("Local Point: ", np.around(local_point,2))
            print("Unit Vector of Interest: ", np.around(uvect_og,2))
            #print("Rotation Vector: ", v)
            #print("Skew Sym Cross-Product: ", v_x)
            #print(np.dot(v_x,v_x))
            print("Rot Matrix:",np.around(rot_matrix,2))
            #print((1/(1+c)))


        ###################
        ## Checks / Debug
        # LP: length-preserving. Success if "1"
        # ACR: confirm sucessfully rotates A onto B. Success if "0"
        if False:
            check_LP  = np.linalg.norm(rot_matrix,2)
            check_ACR = np.linalg.norm(b-np.matmul(rot_matrix,a),2)
            print("|....LP.....|....ACR....|")
            print("|  "+str(check_LP)+"  |  "+str(check_ACR)+"  |")

        return rot_matrix


''' TESTING AND VISUALIZATION BELOW '''


def main():
    """For testing code and visualizing the points"""
    print("--- DEMO CODE: Must enable method (InclinedPlane, SteppedRings ----\n")

    # Collision Box Size, Location, Angle
    dimensions = [0.2, 0.2, 0.2]
    center_loc = [0.5, 0, 0]
    twist_angle = 30

    # Stepped Rings Visualization Demo
    if False:
        print("......Method: SteppedRings ")
        demo_rings = SteppedRings(dimensions,center_loc, np.identity(3))

        from visualizations import plot_path_transforms
        plot_path_transforms(demo_rings._path_tf)

    # Inclined Planes Visualization Demo
    if False:
        print("......Method: InclinedPlane ")
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
            
            a += [np.sin([p["euler"][2]])*np.cos([p["euler"][0]])]
            b += [np.sin([p["euler"][2]])*np.sin([p["euler"][0]])]
            c += [np.cos([p["euler"][2]])]
            
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
