# captures image from USB connected camera and downloads to specified location
import subprocess
import os

gphoto2 = "gphoto2"
photo_directory = "/home/timothy/Desktop/camera" # directory photos are downloaded to
file_name = " --filename %m.%d.%y.%H.%M.%S" #make sure this doesn't repeat names
capture = " --capture-image-and-download"
overwrite = " --force-overwrite"
process_call = gphoto2+file_name+capture+overwrite #can remove overwrite if file_name is iterative "%n"

def subprocess_cmd(command): # allows multiple commands at once
    process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
    proc_stdout = process.communicate()[0].strip()




def capture_photo(): #capture photo and download
    # Use a breakpoint in the code line below to debug your script.
    os.chdir(photo_directory)
    #subprocess.call([gphoto2, process_call])
    #subprocess_cmd("gphoto2 --filename %H.%M.%S --capture-image-and-download --force-overwrite")
    subprocess_cmd(process_call)
    #call("lsusb")


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    capture_photo()


