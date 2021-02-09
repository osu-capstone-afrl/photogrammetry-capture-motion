#!/usr/bin/env python

#####################################################
##          RViz. Pose Array Publisher             ##
#####################################################
# Software License Agreement (BSD License)          #
# Author: Adam Buynak                               #
#####################################################

## IMPORTS ##
import rospy

from path_obj import DetectedObject
from path_obj import InclinedPlane

import geometry_msgs.msg

## Quaternion Tools
from tf.transformations import euler_from_quaternion, quaternion_from_euler

## SUPPORT FUNCTIONS ##
def rosmsg_geoPose(pose):
    ## Recieves dictionary and list formats and returns a tf2.geom.pose message

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
        q_orientGoal = quaternion_from_euler(pose['quaternion'][0],pose['quaternion'][1],pose['quaternion'][2],axes='sxyz')

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

    elif isinstance(pose,list):
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

    else:
        print("---> Incorrect Input Format")

    return pose_goal


## NODES ##
def node_cameraPoseArray(inputArray):
    """ Publish and Latch a Pose Array to a rostopic. """
    
    # Imports
    import rospy
    from geometry_msgs.msg import PoseArray

    # Config node
    pub = rospy.Publisher('cameraPoseArray', PoseArray, queue_size=10) #TODO: couldn't get latch=True to work. Looping instead
    rospy.init_node('cameraPoseArray', anonymous=False)
    rate = rospy.Rate(1) # 10hz

    message = geometry_msgs.msg.PoseArray()
    message.header.frame_id = 'base_link'
    message.poses = inputArray
    
    # Publish node
    while not rospy.is_shutdown():
        #rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()




## MAIN CODE ##
def main():

    # Example detected object definition
    # copied from motion_inclined_plane.py.. duplicate
    object_size = [0.14, 0.06, 0.04]
    object_posn = [0.50, 0.0, 0.4]
    rot_z = 0
    demo_blade = InclinedPlane(object_size, object_posn, rot_z)

    # Generate PoseArray for ROS Node Publisher
    pose_geom = []
    for i in demo_blade.get_positions():
        pose_geom.append(rosmsg_geoPose(i))


    #print(pose_geom)
    #print(pose_geom[0])  #example of single, first pose vector


    # Try launching ros node
    try:
        node_cameraPoseArray(pose_geom)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
