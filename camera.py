# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import subprocess
import os

gphoto2 = "gphoto2"
photo_directory = "/home/timothy/Desktop/camera"
file_name = " --filename %m.%d.%y.%H.%M.%S"
capture = " --capture-image-and-download"
overwrite = " --force-overwrite"
process_call = gphoto2+file_name+capture+overwrite

def subprocess_cmd(command):
    process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
    proc_stdout = process.communicate()[0].strip()




def capture_photo():
    # Use a breakpoint in the code line below to debug your script.
    os.chdir(photo_directory)
    #subprocess.call([gphoto2, process_call])
    #subprocess_cmd("gphoto2 --filename %H.%M.%S --capture-image-and-download --force-overwrite")
    subprocess_cmd(process_call)
    #call("lsusb")


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    capture_photo()


