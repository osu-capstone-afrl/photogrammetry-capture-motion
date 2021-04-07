from robot_support import moveManipulator
from transformations import Transformations
from path_plans import InclinedPlane
from path_plans import SteppedRings
import rospy
import time
import sys
import os
# hacky code to import the camera module
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from camera import capture_photo


if __name__ == '__main__':
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
        tf = Transformations()

        size = [0.14, 0.06, 0.04]
        locator = [0.46, 0.0, 0.32]
        orientation = tf.create_rotation_matrix([0],'z')

        # Add Object to Collision Planning Space
        robot.add_box_object(locator, size)

        # create path plan
        plan = InclinedPlane(size, locator, orientation)
        # plan = SteppedRings(size, locator, orientation)

        # Attempt Incline Plane Motion
        print("\nEXECUTE INCLINED PLANES RASTER MOTION")

        try:
            for msg in plan.path_as_messages:
                robot.goto_Quant_Orient(msg)
                time.sleep(1)
                capture_photo()
                time.sleep(1)
        except KeyboardInterrupt:
            exit()

        ## CLEANUP & CLOSE ##
        raw_input('Remove Object <enter>')
        robot.remove_object()

        raw_input('Return to All-Zeros <enter>')
        robot.goto_all_zeros()

        print "============ Complete!"

    except rospy.ROSInterruptException:
        exit()
    except KeyboardInterrupt:
        exit()

