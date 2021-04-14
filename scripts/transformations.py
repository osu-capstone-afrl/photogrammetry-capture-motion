#!/usr/bin/env python

#####################################################
##   Transformations Support Class                 ##
##                                                 ##
##   Homogeneous Transformation Matrices           ##
##   * Transformation Matrix generation tool       ##
##                                                 ##
#####################################################

# Software License Agreement (Apache 2.0 License)
#
# Copyright (c) 2021, The Ohio State University
# Center for Design and Manufacturing Excellence (CDME)
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
# All rights reserved.
#
# Author: Adam Buynak

#####################################################

import numpy as np
from typing import Union
from typing import List


class Transformations:
    """ Container for Rigid Body Transformations """
    @staticmethod
    def get_transformation(rotation, translation):
        # type: (np.ndarray, Union[List[float], np.ndarray]) -> np.ndarray
        """ Formats separate rotation and translation matrices into one homogeneous transformation
        matrix.

        @param rotation:    3x3 rotational matrix
        @param translation: 3x1 translation vector, list or np.ndarray

        @return: 4x4 homogeneous transformation matrix
        """

        if type(translation) is list:
            translation = np.array(translation)

        if not rotation.shape == (3, 3):
            raise Exception("Error Generating Transformation Matrix. Rotation must be np.ndarray sized 3x3.")
        if not translation.size == 3:
            raise Exception("Error Generating Transformation Matrix. Translation Vector wrong size.")

        translation = translation.reshape((3,1))

        top = np.concatenate((rotation, translation),axis=1)
        bottom = np.array([[0., 0., 0., 1.]])
        transform_matrix = np.concatenate((top, bottom),axis=0)

        return transform_matrix

    @staticmethod
    def create_rotation_matrix(rotations='', order=[]):
        # type: (List[float], str) -> np.ndarray
        """
        Creates a rotation matrix from a list of angles and order. If called without arguments
        it returns the identity matrix. Rotations are done relative to the fixed frame, not the
        current frame.

        @param rotations: List [x, y, z] of how much to rotate on each fixed-frame axis in radians
        @param order:     String showing order to rotate in (e.g., 'xzy' or 'xyz')

        @return: 3x3 rotation matrix
        """
        rotation_matrix = np.identity(3)
        for i, c in enumerate(order):
            theta = rotations[i]
            if c == 'x':
                rot = np.array([[1, 0, 0],\
                                [0, np.cos(theta), -1*np.sin(theta)],\
                                [0, np.sin(theta), np.cos(theta)]])
            elif c == 'y':
                rot = np.array([[np.cos(theta), 0, np.sin(theta)],\
                                [0, 1, 0],\
                                [-1*np.sin(theta), 0, np.cos(theta)]])
            else:
                rot = np.array([[np.cos(theta), -1*np.sin(theta), 0],\
                                [np.sin(theta), np.cos(theta), 0],\
                                [0, 0, 1]])

            rotation_matrix = np.matmul(rot, rotation_matrix)

        return rotation_matrix

    # todo: possibly remove? Decision to be made by @ACBuynak
    # previously convertPath2FixedFrame
    @staticmethod
    def convertPath2FixedFrame(path_body, frame_body, frame_fixed=np.identity((4))):
        """
        Function for mapping a Path (type: List) onto a Fixed Frame (np. Homogenous Matrix)

        @param path_body: Python List of Transformation Matrices for each point along a path
        @param frame_body: Transformation matrix for the body
        @param frame_fixed: Defaults to Identity Matrix (or robot's fixed frame)
        """

        ## Find Arb Frame Defined By Fixed Frame
        frame_b2f = np.matmul(frame_body, frame_fixed)  #Pre-Multiply bc wrt fixed frame

        ## Convert Path Frames to be defined by Fixed Frame
        path_fixed = []
        for point in path_body:
            path_fixed.append(np.matmul(frame_b2f, point))

        return path_fixed

    @staticmethod
    def convert_transformations_to_poses(path_transformations):
        # type: (List[np.ndarray]) -> List[List[float]]
        """
        Converts a path of homogenous transformations to a path of
        poses (x, y, z, qx, qy, qz, qw)

        Function based upon math in...
        https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion

        @param path_transformations: List of Homogeneous Transformations

        @return: List of Robot Poses in the form [x, y, z, qx, qy, qz, qw]
        """
        path_poses = []
        for tf in path_transformations:
            (x, y, z) = tf[0:3, -1]
            (Qxx, Qyy, Qzz, _) = np.diag(tf)
            t = np.trace(tf[:-1, :-1])
            r = np.sqrt(1+t)

            # TODO: Fix (invalid value warning) so copysign quant calculation handles imaginary values (or only uses real?)
            #  assigned to @ACBuynak
            qx = np.copysign(np.absolute(0.5*np.lib.scimath.sqrt(1 + Qxx - Qyy - Qzz)), tf[2, 1]-tf[1, 2])
            qy = np.copysign(np.absolute(0.5*np.lib.scimath.sqrt(1 - Qxx + Qyy - Qzz)), tf[0, 2]-tf[2, 0])
            qz = np.copysign(np.absolute(0.5*np.lib.scimath.sqrt(1 - Qxx - Qyy + Qzz)), tf[1, 0]-tf[0, 1])
            qw = 0.5*r

            qx = 0 if np.isnan(qx) else qx
            qy = 0 if np.isnan(qy) else qy
            qz = 0 if np.isnan(qz) else qz

            path_poses.append([x, y, z, qx, qy, qz, qw])

        return path_poses


# todo Remove or refactor this code. For @ACBuynak
def main():
  print "-- DEMO ONLY --\nACTUAL TOOLS STORED IN 'transformations' CLASS\n"

  tf = Transformations()


  ## Fixed Frame (robot base_link frame)
  frame_fixed = np.identity((4))


  ## Arbitrary Body Frame Variables
  arb_matrix = np.matrix('0 -1 0; 1 0 0; 0 0 1')
  arb_vector = np.matrix('0; 10; 0')

  frame_arb = tf.generateTransMatrix(arb_matrix, arb_vector)
  #print(frame_arb)


  ## Arbitrary Path (defined as Transformation Matrices)
  path_arb = [
    np.matrix('1 0 0 0; 0 1 0 2; 0 0 1 0; 0 0 0 1'),
    np.matrix('1 0 0 1; 0 1 0 2; 0 0 1 0; 0 0 0 1'),
    np.matrix('1 0 0 2; 0 1 0 2; 0 0 1 0; 0 0 0 1'),
    np.matrix('1 0 0 2; 0 1 0 3; 0 0 1 0; 0 0 0 1')
  ]


  ## Map Path to Fixed Frame
  new_path_transforms = tf.convertPath2FixedFrame(path_arb, frame_arb)
  #print('New Fixed Path Generated by tf.convertPath2FixedFrame')
  #print(new_fixed_path)


  ## Convert Path of Transforms to Robot Poses
  new_path_poses = tf.convertPath2RobotPose(new_path_transforms)
  print(new_path_poses)



if __name__ == '__main__':
  main()