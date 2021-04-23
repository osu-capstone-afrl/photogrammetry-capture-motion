#!/usr/bin/env python

import rospy
import subprocess
from photogrammetry_capture_motion.srv import TakePhotoMsg

def handle_take_photo(req):
    
    # Hacky Method to ensure camera usb not claimed by another service. Fix in Future.
    subprocess.call(["pkill", "-f", "gphoto2"])

    # Take Photo
    # 'Overwrite' = can remove overwrite if file_name is iterative "%n"
    file_name = " --filename " + req.filepath  # make sure this doesn't repeat names
    subprocess.call(["gphoto2", "--capture-image-and-download", "--filename", req.filepath, "--force-overwrite"])
    
    return TakePhotoMsg(True)


def cam_control_server():
    rospy.init_node('d5600_take_photo')
    s = rospy.Service('d5600_take_photo', TakePhotoMsg, handle_take_photo)
    print("Ready to recieve camera capture commands.")
    rospy.spin()

if __name__ == "__main__":
    cam_control_server()