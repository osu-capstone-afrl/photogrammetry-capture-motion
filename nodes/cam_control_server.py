#!/usr/bin/env python

import rospy
import subprocess
from photogrammetry_capture_motion.srv import TakePhotoMsg

def handle_take_photo(req):
    file_name = " --filename " + req.filepath  # make sure this doesn't repeat names
    gphoto2 = "gphoto2"
    capture = " --capture-image-and-download"
    overwrite = " --force-overwrite"
    process_call = gphoto2 + capture + file_name + overwrite  # can remove overwrite if file_name is iterative "%n"
    subprocess.call(["gphoto2", "--capture-image-and-download", "--filename", req.filepath, "--force-overwrite"])
    
    return TakePhotoMsg(True)


def cam_control_server():
    rospy.init_node('d5600_take_photo')
    s = rospy.Service('d5600_take_photo', TakePhotoMsg, handle_take_photo)
    print("Ready to recieve camera capture commands.")
    rospy.spin()

if __name__ == "__main__":
    cam_control_server()