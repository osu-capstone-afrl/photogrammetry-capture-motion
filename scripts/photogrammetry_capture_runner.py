#!/usr/bin/env python

from robot_support import moveManipulator
from path_plans import InclinedPlane
from path_plans import SteppedRings
from path_plans import OrthogonalCapture
from utils import get_path_from_json
import numpy as np
import geometry_msgs.msg
import argparse
import rospy
import time
import json
import sys
import os
from photogrammetry_capture_motion.srv import TakePhotoMsg
# hacky code to import the camera module
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
#from camera import capture_photo


def cam_control_client(filepath):
    print("Wait for Service")
    rospy.wait_for_service('d5600_take_photo')
    try:
        capture_photo = rospy.ServiceProxy('d5600_take_photo', TakePhotoMsg)
        resp = capture_photo(filepath)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Input JSON file name")
    parser.add_argument('-json_name', '--json-name', type=str, default='detected_object.json')
    args = parser.parse_args()
    json_name = args.json_name

    # Debug Settings
    param_take_photos = True  # If True, use 'gphoto2' to take photos (via a ROS Service)

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
        robot = moveManipulator(input_planning_group = "eef_camera")
        robot.set_accel(0.1)
        robot.set_vel(0.1)

        # Move to Known Start Position: All-Zeros
        raw_input('Go to All-Zeros Position <enter>')
        robot.goto_all_zeros()

        ## PLANNING ##
        # Example detected object definition
        fname = os.path.join(current, json_name)
        with open(fname, "r") as read_file:
            detected_object = json.load(read_file)

        path = get_path_from_json(detected_object)

        object_size = detected_object['ObjectInfo']['size']
        object_posn = detected_object['ObjectInfo']['position']
        orientation = np.array(detected_object['ObjectInfo']['orientation'])

        # Add Object to Collision Planning Space
        robot.add_box_object(object_posn, object_size)

        # Attempt Incline Plane Motion
        print("\nEXECUTE INCLINED PLANES RASTER MOTION")

        filepath = os.path.join(parent,'photos/test_photo.png')

        try:
            for i, msg in enumerate(path.path_as_messages):
                print(">> SEND POSITION")
                robot.goto_Quant_Orient(msg)
                time.sleep(1)

                if param_take_photos:
                    print(">> ATTEMPT TO TAKE PHOTO")
                    filepath = os.path.join(parent,'photos/test_photo_%s.jpg' % (i))
                    cam_control_client(filepath)
                    print(">> PHOTO TAKEN")
                    # time.sleep(1)
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

