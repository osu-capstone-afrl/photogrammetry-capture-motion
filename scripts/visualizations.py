#!/usr/bin/env python

#####################################################
##   Visualizations Support Class                  ##
##                                                 ##
##   Construct Visuals                             ##
##   * Show path in 3D space                       ##
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
import matplotlib.pyplot as plt


def plot_path_transforms(d2p):
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Origin
    ax.scatter(0, 0, 0)  # origin

    # Collect Array of Just Vectors
    vectors = np.zeros((len(d2p),3))
    for i, transform in enumerate(d2p):
        vectors[i, :] = transform[:-1, 3]

    # print(vectors)

    # Path Plotted
    for p in d2p:
        ax.scatter(p[0, 3], p[1, 3], p[2, 3], marker="x")
    ax.plot(vectors[:, 0], vectors[:, 1], vectors[:, 2], color='b')

    # Assign Labels
    ax.set_xlabel('X-Axis')
    ax.set_ylabel('Y-Axis')
    ax.set_zlabel('Z-Axis')

    # Display Plot
    plt.show()

    # Close Figure
    #plt.close()
