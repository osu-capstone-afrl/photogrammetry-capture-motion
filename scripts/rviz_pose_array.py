#!/usr/bin/env python

#####################################################
##          RViz. Pose Array Publisher             ##
#####################################################
# Software License Agreement (BSD License)          #
# Author: Adam Buynak                               #
#####################################################

import numpy as np
from path_plans import DetectedObject
from path_plans import InclinedPlane
from path_plans import SteppedRings
from path_plans import OrthogonalCapture
from transformations import Transformations
import json
import os
import rospy
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler


#####################################################
## SUPPORT FUNCTIONS ##
def rosmsg_geoPose(pose):
    ## Receives dictionary and list formats and returns a tf2.geom.pose message

    # Get Current Orientation in Quanternion Format
    # http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html
    #q_poseCurrent = self.move_group.get_current_pose().pose.orientation
    #print(q_poseCurrent)

    # Using Quaternion's for Angle
    # Conversion from Euler(rotx,roty,rotz) to Quaternion(x,y,z,w)
    # Euler Units: RADIANS
    # http://docs.ros.org/en/melodic/api/tf/html/python/transformations.html
    # http://wiki.ros.org/tf2/Tutorials/Quaternions
    # http://docs.ros.org/en/api/geometry_msgs/html/msg/Quaternion.html

    if isinstance(pose,dict):
        q_orientGoal = quaternion_from_euler(pose['euler'][0],pose['euler'][1],pose['euler'][2],axes='sxyz')

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose['position'][0]
        pose_goal.position.y = pose['position'][1]
        pose_goal.position.z = pose['position'][2]
        #pose_goal.orientation.x = pose['quaternion'][0]
        #pose_goal.orientation.y = pose['quaternion'][1]
        #pose_goal.orientation.z = pose['quaternion'][2]
        #pose_goal.orientation.w = pose['quaternion'][3]

        #Temp fix before converting eulers
        pose_goal.orientation.x = q_orientGoal[0]
        pose_goal.orientation.y = q_orientGoal[1]
        pose_goal.orientation.z = q_orientGoal[2]
        pose_goal.orientation.w = q_orientGoal[3]

    elif isinstance(pose,list) and len(pose)==6:
        # Accepts List of Length 6.
        # List = [ x, y, z, euler_x, euler_y, euler_z ]
        
        # Convert Euler Orientation Request to Quanternion
        q_orientGoal = quaternion_from_euler(pose[3],pose[4],pose[5],axes='sxyz')

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.x = q_orientGoal[0]
        pose_goal.orientation.y = q_orientGoal[1]
        pose_goal.orientation.z = q_orientGoal[2]
        pose_goal.orientation.w = q_orientGoal[3]

    elif isinstance(pose, list) and len(pose) == 7:
        # Accepts List of Length 7.
        # List = [ x, y, z, q_x, q_y, q_z, q_w ]
    
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.x = pose[3]
        pose_goal.orientation.y = pose[4]
        pose_goal.orientation.z = pose[5]
        pose_goal.orientation.w = pose[6]
    
    else:
        print("\n---> WARNING: Incorrect Input Format\n")

    return pose_goal


## NODES ##
def node_cameraPoseArray(inputArray):
    """ Publish and Latch a Pose Array to a rostopic. """
    
    # Imports
    import rospy
    from geometry_msgs.msg import PoseArray

    # Config node
    pub = rospy.Publisher('cameraPoseArray', PoseArray, queue_size=10) #TODO: couldn't get latch=True to work. Looping instead
    rospy.init_node('cameraPoseArray', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    message = geometry_msgs.msg.PoseArray()
    message.header.frame_id = 'base_link'
    message.poses = inputArray
    
    # Publish node
    while not rospy.is_shutdown():
        #rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()



#####################################################
## MAIN CODE ##
def main():    
    # Example detected object definition
    if False:
        tf = Transformations()
        object_size = [0.06, 0.14, 0.14]
        object_posn = [0.48, 0.0, 0.32]

        orientation = tf.create_rotation_matrix([0],'z')

        ## Sample Use: Inclined Plane
        # demo_blade = InclinedPlane(object_size, object_posn, np.identity(3), count=(3,3), slope=0.2, clearance=0.06, offset=0.02)

        ## Sample Use: Stepped Rings
        demo_blade = SteppedRings(object_size, object_posn, orientation, scale=1.01, offset=0.01, level_count=2, density=7)
    else:
        current = os.path.dirname(os.path.realpath(__file__))
        fname = os.path.join(current, "detected_object.json")
        with open(fname, "r") as read_file:
            detected_object = json.load(read_file)

        object_size = detected_object['size']
        object_posn = detected_object['posn']
        orientation = np.array(detected_object['orientation'])

        if detected_object['type'] == 'steppedrings':
            demo_blade = SteppedRings(object_size, object_posn, orientation) #, scale=1.01, offset=0.01, level_count=2, density=7)
        else:
            print 'Warning: invalid path type in rviz_pose_array.py'



    ## Visualization in RVIZ
    # Generate PoseArray for ROS Node Publisher
    pose_geom = [rosmsg_geoPose([object_posn[0],object_posn[1],object_posn[2],0,0,0,0])]
    for i in demo_blade.path_as_poses:
        pose_geom.append(rosmsg_geoPose(i))

    # Try launching ros node
    try:
        node_cameraPoseArray(pose_geom)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
