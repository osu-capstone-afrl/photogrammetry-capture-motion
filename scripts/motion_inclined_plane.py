#!/usr/bin/env python

#####################################################
##      Photogrammetry Photos Capture Tool         ##
##                                                 ##
##   Method: Inclined Planes                       ##
##   * This is a ROS enabled node.                 ##
##   * Initial Prototype                           ##
##                                                 ##
#####################################################

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, The Ohio State University
# All rights reserved.
#
# Authors: Adam Buynak, Andrew Schellenberg

#####################################################

### IMPORTS
#
# `moveit_commander` namespace allows Python MoveIt interfaces.
# Includes a `MoveGroupCommander`_, `PlanningSceneInterface`_, and `RobotCommander`_ class
# 
# Additional imports allow used for support, ROS messages, and etc.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from motoman_msgs.srv import ReadSingleIO, WriteSingleIO
import time

# Capstone specific imports
from path_plans import DetectedObject
from path_plans import InclinedPlane

## Quaternion Tools
from tf.transformations import euler_from_quaternion, quaternion_from_euler


#####################################################
## SUPPORT CLASSES AND FUNCTIONS
##
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class moveManipulator(object):
    """moveManipulator Class"""
    def __init__(self):
        super(moveManipulator, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_moveManipulator', anonymous=True)

        # Setup Variables needed for Moveit_Commander
        self.glider_name = ''
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "bot_gp7"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

    def set_vel(self,max_vel):
        ## Wrapper for Moveit Commander's max_velocity
        ## Allowed range...   0.0 <= max_vel <= 1.0
        self.move_group.set_max_velocity_scaling_factor(max_vel)

    def set_accel(self,max_accel):
        ## Wrapper for Moveit Commander's max_acceleration
        ## Allowed range...   0.0 <= max_vel <= 1.0
        self.move_group.set_max_acceleration_scaling_factor(max_accel)

    def goto_all_zeros(self):
        ## Go to "ALL-Zeros" position
        ## Get Current Position & Go to "All-Zeros" Position
        ## Trajectory Type: JOINT MOTION defined by joint position

        # Get Current Position
        joint_goal = self.move_group.get_current_joint_values()

        # Define "All-Zeros" Position
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0

        # Send action to move-to defined position
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def plan_cartesian_throw_path(self, scale=1):
        ## Plan Cartesian Path to throw glider

        # Specify a list of waypoints
        waypoints = []

        # Example - Commented Out
        #wpose = self.move_group.get_current_pose().pose
        #wpose.position.z += scale * 0.1  # Move up (z)
        #wpose.position.x += scale * 0.8  # Forward (x)
        #waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 5 cm
        # which is why we will specify 0.05 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints,   # waypoints to follow
                                                                  0.05,        # eef_step
                                                                  0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def goto_Quant_Orient(self,msg_geoPose):
        ## GOTO Pose Using Cartesian + Quaternion Pose

        pose_goal = self.rosmsg_geoPose(msg_geoPose)

        self.move_group.set_pose_target(pose_goal)

        ## Call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        # For testing:
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def rosmsg_geoPose(self,pose):
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
            q_orientGoal = quaternion_from_euler(pose['euler'][0],pose['euler'][1],pose['euler'][2],axes='sxyz')

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = pose['position'][0]
            pose_goal.position.y = pose['position'][1]
            pose_goal.position.z = pose['position'][2]
            #pose_goal.orientation.x = pose['quaternion'][0]
            #pose_goal.orientation.y = pose['quaternion'][1]
            #pose_goal.orientation.z = pose['quaternion'][2]
            #pose_goal.orientation.w = pose['quaternion'][3]

            # Reorganize Data into Output Format
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

    def goto_joint_posn(self,joint_goal):
        ## Go to Joint Defined position
        ## Get Current Position & Go to "All-Zeros" Position
        ## Trajectory Type: JOINT MOTION defined by joint position

        # Send action to move-to defined position
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def add_box_object(self, cart_location, size, timeout=4):
        ## Add object cube Element to Collision Scene

        # Create object
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = "base_link"
        object_pose.pose.orientation.w = 1.0
        object_pose.pose.position.x = cart_location[0]
        object_pose.pose.position.y = cart_location[1]
        object_pose.pose.position.z = cart_location[2]
        self.object_name = "box_" + str(cart_location[0]) + str(cart_location[1]) + str(cart_location[2])

        # Add object to scene
        self.scene.add_box(self.object_name, object_pose, size=(size[0], size[1], size[2]))

        # Alternively, Use Mesh of Object. (mixed success with this. See moveit webpage)
        # self.scene.add_mesh(self.object_name, object_pose, filename="$(find object)/meshes/object-model.stl", size=(1,1,1))

    def add_sphere_object(self, cart_location, size, timeout=4):
        ## Add object sphere Element to Collision Scene

        # Create object
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = "base_link"
        object_pose.pose.orientation.w = 1.0
        object_pose.pose.position.x = cart_location[0]
        object_pose.pose.position.y = cart_location[1]
        object_pose.pose.position.z = cart_location[2]
        self.object_name = "sphere_" + str(cart_location[0]) + str(cart_location[1]) + str(cart_location[2])

        # Add object to scene
        self.scene.add_sphere(self.object_name, object_pose, radius=size)

    def remove_object(self, timeout=4):
        ## Removing Objects from the Planning Scene
        ## **Note:** The object must be detached before we can remove it from the world
        self.scene.remove_world_object(self.object_name)

    def act_gripper(self, request):
        ## Wrapper for rosservice to open/close gripper using Read/Write IO

        # Wait for ros services to come up
        rospy.wait_for_service('read_single_io')
        rospy.wait_for_service('write_single_io')

        # Create Handle for Service Proxy's
        try:
            read_single_io = rospy.ServiceProxy('read_single_io', ReadSingleIO)
            write_single_io = rospy.ServiceProxy('write_single_io', WriteSingleIO)
        except rospy.ServiceException as e:
            print("Gripper IO Service Call failed: %s"%e)

        # Send 'Write' IO Message
        try:
            write_status = write_single_io(10010, request)
        except:
            print("An exception occured. Unable to write to Single IO.")


        # Call Read Service to check current position
        read_status = read_single_io(10011).value
        if read_status:
            print('Gripper is Closed')
        else:
            print('Gripper is Open')

        return read_status

    def execute_plan(self, plan):
        ## Execute a Plan
        ## Use execute if you would like the robot to follow a plan that has already been computed:
        self.move_group.execute(plan, wait=True)

    def wait_for_state_update(self, glider_is_known=False, glider_is_attached=False, timeout=4):
        ## wait_for_scene_update
        ## Serves to ensure that the paper-airplane simulated object has been attached to simulated robot model
        ## This helps with collision planning. Not as important for paper-glider project, but is best practice.

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the glider is in attached objects
            attached_objects = self.scene.get_attached_objects([self.glider_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the glider is in the scene.
            # Note that attaching the glider will remove it from known_objects
            is_known = self.glider_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (glider_is_attached == is_attached) and (glider_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False







def main():
    try:
        print "----------------------------------------------------------"
        print "            Photogrammetry Image Capture Tool             "
        print "----------------------------------------------------------"
        print ""
        print "Program Developed by ACBUYNAK, SCHELLENBERG3. Spring 2021"
        print "Press Enter to advance script when prompted."
        print ""



        ## SETUP ##
        # Initial Values & Controls
        robot = moveManipulator()
        robot.set_accel(0.2)
        robot.set_vel(0.2)

        # Move to Known Start Position: All-Zeros
        #raw_input('Go to All-Zeros Position <enter>')
        robot.goto_all_zeros()



        ## PLANNING ##
        # Example detected object definition
        object_size = [0.14, 0.06, 0.04]
        object_posn = [0.50, 0.0, 0.4]
        rot_z = 0
        demo_blade = InclinedPlane(object_size, object_posn, rot_z)

        # Add Object to Collision Planning Space
        robot.add_box_object(object_posn, object_size)

        ## MOTION EXECUTION ## 
        #~~ Check Current Pose
        #print(robot.move_group.get_current_pose().pose)

        # Attempt Incline Plane Motion
        print("\nEXECUTE INCLINED PLANES RASTER MOTION")
        radius = 0.005

        try:
            for msg in demo_blade.pose_and_orientation:  #Debugging. Only doing first 5 poses poseList[0:5]
                print(msg)
                # todo: @Adam to make updates this loop and verify that it pulls the position
                # todo: and orientation properly
                # robot.add_sphere_object(msg["position"], radius)
                robot.goto_Quant_Orient(msg)
                time.sleep(0.2)
        except KeyboardInterrupt:
            return

        ## CLEANUP & CLOSE ##
        # Remove Object
        raw_input('Remove Object <enter>')
        robot.remove_object()

        raw_input('Return to All-Zeros <enter>')
        robot.goto_all_zeros()

        print "============ Complete!"

    except rospy.ROSInterruptException:
        return

    except KeyboardInterrupt:
        return


if __name__ == '__main__':
  main()
