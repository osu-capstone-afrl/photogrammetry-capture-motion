# Capture Tool for Photogrammetry Image Set :camera:
Support package for capturing images from a DSLR camera mounted to the flange of a Motoman GP7. Intended to support a undergraduate capstone project on robotic inspection applications. Developed at The Ohio State University.

_Note: This repository was designed for ROS Melodic. It has not been tested on other distributions._
_Specifically designed for the Motoman GP7 robot as supported by the ROS-Industrial program._

## Tools Contained
* Inclined Plane Motion Method (for Motoman GP7)

## Installation
The following section explains how to setup the package to work.

### Prerequisites
  - **ROS Melodic:** For obtaining and configuring ROS follow the installation instructions for [full-desktop Melodic installation](http://wiki.ros.org/melodic/Installation/Ubuntu).
  - **Catkin workspace:** Create a clean [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/index.html) workspace.
  - **MoveIt 1:** For installation instructions see [MoveIt's webpage](https://moveit.ros.org/install/).

### Required Repositories
  Clone the below repositories into your catkin-tools workspace:
  - [photogrammetry-capture-motion](https://github.com/osu-capstone-afrl/photogrammetry-capture-motion)
  - [ros-industrial/motoman](https://github.com/ros-industrial/motoman)


### Dependencies
To automatically install any missing dependencies of your ROS installation, run the following terminal commands:

```shell
#---------- install third party dependencies -----------
sudo apt-get update
# Move to the root of the workspace
cd [PATH/TO/YOUR/WORKSPACE]
# Install all dependencies of packages in the workspace
rosdep install --from-paths src --ignore-src -r -y
# Install all python dependencies
pip install -r src/photogrammetry-capture-motion/requirements.txt
# Build your workspace
catkin build
source devel/setup.bash
```
Once the workspace build process is completed you are ready to start playing...cheers!

#### Install missing dependencies
If the build fails, it occurs usually to missing package dependencies or missing third party (non-ros) packages. When this occurs the build log in the terminal indicates the name of the package dependency that it is missing, then try:

```shell
sudo apt-get update ros-kinetic-[package-name]
# separate the package name words with a '-'
```
If a package is not found it is probably a third-party dependency, google the name of the package and search for installation instructions:

## Usage

```shell
roslaunch motoman_gp7_moveit_config moveit_planning_execution.launch sim:=true
rosrun photogrammetry-capture-motion motion_inclined_plane.py
```
