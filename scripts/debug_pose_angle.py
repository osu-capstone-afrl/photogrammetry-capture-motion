#!/usr/bin/env python

#####################################################
##                DEBUG POSE ANGLE                 ##
#####################################################

import numpy as np
import path_plans
from rviz_pose_array import rosmsg_geoPose
from rviz_pose_array import node_cameraPoseArray

import rospy
import geometry_msgs.msg

import time
from math import trunc



def _find_rot_matrix(local_point,fixed_origin,angle):

    #Convert from Degrees to Radians
    # deg2rad(x) = angle * pi / 180.
    angle = np.deg2rad(angle)

    # World Frame (ignoring translation)
    vect_og = np.subtract(fixed_origin, local_point)
    uvect_og = vect_og / np.linalg.norm(vect_og)

    # New Tool Frame
    uvect_z = np.array([0,0,1])

    ###################
    ## METHOD 3
    # Based on https://math.stackexchange.com/a/476311
    if True:
        # Find ROT_MATRIX necessary to rotation vector 'a' onto 'b'
        a = uvect_z     # unit z vector
        b = uvect_og    # point to center

        v = np.cross(a,b)
        v = v / np.linalg.norm(v,2)
        print('Rotate about v:',np.around(v,3))

        #c = np.dot(a,b)                      # cos(pheta)
        #s = np.linalg.norm(np.cross(a,b),2)  # sin(pheta)
        c = np.cos(angle)                     #angle in radians
        s = np.sin(angle)

        #DEBUG: Output cos & sin values to determine if quantrant issues.
        print('cos:',np.around(c,3))
        print('sin:',np.around(s,3))

        v_x = np.array( [[ 0,      -v[2],   v[1] ],
                         [ v[2],    0,     -v[0] ],
                         [-v[1],    v[0],   0    ]] )

        v_tensor = np.array([[v[0] ** 2, v[0] * v[1], v[0] * v[2]],
                             [v[0] * v[1], v[1] ** 2, v[1] * v[2]],
                             [v[0] * v[2], v[1] * v[2], v[2] ** 2]])
        
        rot_matrix = c*np.identity(3) + s*v_x + (1-c)*v_tensor

    return rot_matrix


## MAIN CODE ##
def main():

    from transformations import transformations
    tf = transformations()

    # Center Part "_locator"
    fixed_posn = [0, 0, 0]
    fixed_rot  = np.identity(3)
    fixed_pose = tf.generateTransMatrix(fixed_rot,fixed_posn)

    # Set Camera Cartesian Point
    x = -0.25
    y = 0
    z = 0.25

    angles = np.linspace(-179,179,359)
    #angles = [135]

    # Lists for Visualization
    theta_actual_cos = []
    theta_actual_sin = []

    try:
        # Imports
        import rospy
        from geometry_msgs.msg import PoseArray

        # Config node
        pub = rospy.Publisher('cameraPoseArray', PoseArray, queue_size=10) #TODO: couldn't get latch=True to work. Looping instead
        rospy.init_node('cameraPoseArray', anonymous=False)
        rate = rospy.Rate(1) # 10hz

        # First Message
        pose_geom = [rosmsg_geoPose([0,0,0,0,0,0])]
        message = geometry_msgs.msg.PoseArray()
        message.header.frame_id = 'base_link'
        message.poses = pose_geom
        
        # Publish node
        pub.publish(message)
        rate.sleep()

        # Loop Through Angles
        for ph in angles:
            print('...............................Angle Input to Shown Pose:' + str(ph))

            # Create Local Point's Transform Matrix
            local_rot_matrix = _find_rot_matrix([x,y,z],fixed_origin=fixed_posn,angle=ph)
            local_transform = tf.generateTransMatrix(local_rot_matrix,[x,y,z])

            # Convert to Fixed Frame (doesn't really do anything since already at origin)
            fixed_transform = np.matmul(fixed_pose, local_transform)
            print(np.around(fixed_transform,3))

            # Check Rotation Matrix
            print('')
            theta_c = np.rad2deg( np.arccos(fixed_transform[0][0]) )
            theta_actual_cos.append(theta_c)
            print('Theta_c',np.around(theta_c,3) )

            theta_s = np.rad2deg( np.arcsin(fixed_transform[0][2]) )
            theta_actual_sin.append(theta_s)
            print('Theta_s',np.around(theta_s,3) )

            # Convert path to Robot Poses (outputs a list of vectors x,y,z,qx,qy,qz,qw)
            _path_pose = tf.convertPath2RobotPose([fixed_transform])

            # Add to PoseArray for ROS Node Publisher
            pose_geom = [rosmsg_geoPose([0,0,0,0,0,0])]
            for i in _path_pose:
                pose_geom.append(rosmsg_geoPose(i))

            message = geometry_msgs.msg.PoseArray()
            message.header.frame_id = 'base_link'
            message.poses = pose_geom
            pub.publish(message)

            time.sleep(0.02)

    except rospy.ROSInterruptException:
        pass


    # Plot Array Differences
    if len(angles)>1:
        import matplotlib.pyplot as plt
        x = range(0,len(angles))

        fig, (ax1, ax2) = plt.subplots(1,2)

        fig.suptitle('Rotation Matrix Debugging Analysis\n ')

        ax1.plot(angles,angles,label = 'Theta - Input')
        ax1.plot(angles,theta_actual_cos, 'g:', label = 'Theta - Cos derived (cell [0,0])')
        ax1.plot(angles,theta_actual_sin, 'b-',label = 'Theta - Sin derived (cell [0,2])')
        ax1.grid()
        ax1.legend()
        ax1.set_xlabel('Input Angle (degrees)')
        ax1.set_ylabel('Output Angle (degrees)')
        ax1.set_title('Evaluation of Rotation Matrix (in/out theta values)')

        delta_theta_input2sin = np.subtract( np.array(theta_actual_sin) , np.array(theta_actual_cos) )
        ax2.plot(angles,delta_theta_input2sin)
        ax2.grid()
        ax2.set_xlabel('Input Angle (degrees)')
        ax2.set_ylabel('Delta (degrees)')
        ax2.set_title('Differences btwn Cos- & Sin- Angles Derivatives')

        plt.show()


if __name__ == '__main__':
    main()
