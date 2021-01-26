#!/usr/bin/env python

#####################################################
##        Stream Current Robot Position            ##
##                                                 ##
##   * Joint Position                              ##
##   * Cartesian Position                          ##
##                                                 ##
#####################################################

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, The Ohio State University
# All rights reserved.
#
# Author: Adam Buynak

#####################################################

### IMPORTS
#
# `moveit_commander` namespace allows Python MoveIt interfaces.
# Includes a `MoveGroupCommander`_, `PlanningSceneInterface`_, and `RobotCommander`_ class
#
# Additional imports allow used for support, ROS messages, and etc.

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
import time


#####################################################
## SUPPORT CLASSES AND FUNCTIONS
##
class moveManipulator(object):
    """moveManipulator Class"""
    def __init__(self):
        super(moveManipulator, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_moveManipulator', anonymous=True)

        # Setup Variables needed for Moveit_Commander
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




def main():
    try:
        print "----------------------------------------------------------"
        print "            Photogrammetry Image Capture Tool             "
        print "----------------------------------------------------------"
        print ""
        print "Program Developed by ACBUYNAK. Spring 2021"
        print "Press Enter to advance script when prompted."
        print ""

        # Initial Values & Controls
        robot = moveManipulator()

        # Loop for 5 Seconds
        try:
           for i in range(0,5):
             pose_joints = robot.move_group.get_current_joint_values()
             pose_cart = robot.move_group.get_current_pose().pose
             print(pose_joints)
             print(pose_cart)
             time.sleep(1)
        except KeyboardInterrupt:
          sys.exit()



        print "============ Complete!"

    except rospy.ROSInterruptException:
        return

    except KeyboardInterrupt:
        return


if __name__ == '__main__':
  main()
