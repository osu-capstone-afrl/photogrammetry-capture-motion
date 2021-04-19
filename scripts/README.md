# Scripts

The the code in `scripts/` is to generate a path for the robot to follow which properly positions and orients the a camera for photographing a part in its workspace.  These paths are generated based on an estimate of the object's largest dimensions and center location.



## Intended Use

When writing a program, here's how to use one of the generated path plans. In your main python file (e.g., `motion_inclined_plane.py` in this case) simply include `from path_plans import YOUR_DESIRED_PATH_PLAN` .  Each path plan requires:

1. The parts size as a `List[x_dimension, y_dimension, z_dimension]` in meters
2. The parts center location as a `List[x_coordinate, y_coordinate, z_coordinate]` in meters
3. The parts orientation as a `float` in degrees

Each path has some unique variables to adjust their set up. The all have default values that can be overwritten. For more information please check out the docstrings under each path class in the code.

The base of the coordinate system is the same as the robot's base frame.  Orientation is measured between the part's x-axis and the robots. See [Details: Assumptions](#Assumptions) for more on this.

To get use a path plan in code might look like this...

```python
from pathplans import InclinedPlane

object_size = [0.1, 0.05, 0.02]  # meters
object_posn = [0.25, 0.0, 0.01]  # meters
object_rot = 30  # Degrees

to_photograph = InclinedPlane(object_size,
                              object_posn,
                              object_rot,
                             #x_num=7,   # This is where one would specify the value
                             #z_num=10,  # for a path plan's unique set up options
                             ) 

for message in to_photograph.pose_andorientation:
    print message
```

 Once instantiated the pose and orientation are calculated, formatted as a ROS message, and stored as a list in the member variable `pose_and_orientation`.  This process is the same for any path plan.

Unless writing a new path plan or making custom changes there should not be any need to call member functions or access and other member variables.



## Description of Path Generation

All paths are generated following the same basic steps. In more detail, this is what really happens when running the code example in [Intended Use](#Intended-Use)

When instantiated, positions are generated first.  This is based on the the geometry information provided in `object_size` and is done with respect to the robot's base frame. Rather than just `[x, y, z]` coordinates, we define each as a [homogenous transformation](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/) relative to the robot's base frame.  We now have a list of 4x4 homogenous matrices.

After this a second transformation is applied to each matrix, this rotates them to match `object_rot` and translates them to center around `object_posn`. 

Finally, the z-axis of each frame should point towards the part. This orientation is challenging and the method for determining this transformation for each point is ongoing development. 



### Inclined Plane

This path plan consists of four planes, one for each side of the part's bounding box. The length of each plan matches the side of the bounding box that is is on and the height of all planes is equal to the height of the bounding box. The planes are sloped in, towards the box. 

```python
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
```



### Stepped Rings

This path plan consists of a several circles stacked, equally spaced, in the z-direction. Poses are evenly distributed on the circumference of each circle.

```python
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
```



## Details 

The following table briefly summarizes the contents of this directory:

| File Name                                                    | Contents                                                     |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [`motion_inclined_plane.py`](https://github.com/osu-capstone-afrl/photogrammetry_capture_motion/blob/melodic/scripts/motion_inclined_plane.py) | Demonstration of code                                        |
| [`path_plans.py`](https://github.com/osu-capstone-afrl/photogrammetry_capture_motion/blob/melodic/scripts/path_plans.py) | Contains all path plans                                      |
| [`shapes.py`](https://github.com/osu-capstone-afrl/photogrammetry_capture_motion/blob/melodic/scripts/shapes.py) | Defines basic geometries (e.g., planes) which path plans are built from |
|  |  |
| [`transformations.py`](https://github.com/osu-capstone-afrl/photogrammetry_capture_motion/blob/melodic/scripts/transformations.py) | Tool Class for creating, manipulating, and converting Homogeneous Transformation Matrixes (and list's thereof)     |
| [`visualizations.py`](https://github.com/osu-capstone-afrl/photogrammetry_capture_motion/blob/melodic/scripts/visualizations.py) | 3D Plots of Generated Paths using list of H.Transforms. Powered by MatPlotLib.
| [`rviz_pose_array.py`](https://github.com/osu-capstone-afrl/photogrammetry_capture_motion/blob/melodic/scripts/rviz_pose_array.py) | ROS Node for publishing a 'pose array' object. Array composed of all frames in generated path.

Any unlisted files are safe for users to ignore. They are likely left over testing or debugging code that should be removed eventually. 



### Assumptions 

The code is intended to generate a good path plan regardless of the part's position and orientation.  However, we believe that some methods may be best if the object is placed in an expected manner. Thus we recommend: 

*  When using an `InclinedPlane` path plan recommend orienting a part with its longest dimension along the robot's x-axis.

These will be updated as the code is developed and physical testing occurs.



### Terminology

In robotics terms are often nuanced and dependent on the context or method used. For example, pose, position, location, and orientation are all related terms that could describe something in space or how the robot is configured to reach a point. 

For consistency and clarity, we highlight some terms here and explain what they generally refer to in the code.

* **Center** location of the center relative to the robot's base frame in meters.
* **Frame** or coordinate frame. A right-hand-rule reference frame usually described by a homogenous transformation matrix from the robot's base frame.
* **Homogenous Transformation** a 4x4 matrix encoding rotation and transformation from one coordinate frame to another.
* **Locator** refers to the part's position relative to the robot's base frame.
* **Message** a list of position in `[x, y, z]` and orientation in quaternion `[qx, qy, qz, qw]` used by ROS.
* **Orientation** a rotation matrix, often part of a homogenous transformation.
* **Path Plan** a [`path_plans.py`](https://github.com/osu-capstone-afrl/photogrammetry_capture_motion/blob/melodic/scripts/path_plans.py) object or the list of messages held in the object's `pose_andorientation` member variable.
* **Pose** in this code refers exclusively to the combination of a position and orientation needed to describe a frame.  Poses are formatted into messages for ROS.
* **Position** abbreviated as posn. a list of `[x, y, z]`, often part of a homogenous transformation.
* **Rotation** the angle between the robot's x-axis and the part's x-axis in degrees.
* **Size** maximum dimensions of the part in meters.



### Code Conventions

In short, following these conventions will eliminate redundancy, increase readability, and enable collaboration. Writing minimal, modular code is a focus.   

* **PEP 8** should be followed as a default. See [here](https://www.python.org/dev/peps/pep-0008/#introduction) for the complete guide or [here](https://realpython.com/python-pep8/) for a better explanation and reference. [PyCharm](https://www.jetbrains.com/pycharm/) makes this easy and automatically tracks this formatting so it should be easy to maintain.  
* **Spacing** four whitespaces should be used at each level. Not a tab.
* **Comments** code should be clear enough to read with minimal comments. Descriptive variable names, functions, typing, and docstrings should provide enough context. Active debugging and and short explanations for truly elusive sections of code are the exception.

* **Docstrings** should always be used, even for member functions. Classes should have a short description and their initialization function should be typed and contain a description. See the example below

  ```python
  class MyClass(DetectedObject):
      """A container for the incline plane path plan"""
      def __init__(self, size, center, orientation, x_num=5, z_num=5, slope=0.5, offset=0.25):
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
  ```

* **New path plans** should all inherit from the `DetectedObject` class which holds the size, rotation, and location of a detected object and establishes the `.pose_and_orientation`  variable. The job of each class is to to populate the pose and orientation variable.



