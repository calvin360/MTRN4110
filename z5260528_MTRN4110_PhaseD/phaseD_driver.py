# using the automarker for phase b as the base of this implmentation

import re
import subprocess
import time
# import cv2
# import numpy as np
# import matplotlib.pyplot as plt 
from helperC import *

exeFileName = "phase_d.exe"
webotsAppLocation = "\"C:\\Program Files\\Webots\\msys64\\mingw64\\bin\webots.exe\""
worldFileName = "z1234567_MTRN4110_PhaseA.wbt"
RED_TEXT = '\033[91m'
END_TEXT_FMT = '\033[0m'

MAZE_FILE_NAME = "Maze_1.png"
IMAGE_LADYBUG_FILE_NAME = "Ladybug_small.png"
ROBOT_FILE_NAME = "Robot_1.png"
MAP_FILE_NAME = "Map.txt"

imgRGB = display(MAZE_FILE_NAME)
#Detect corners
mag,cyan = detect_corners(imgRGB) 
corners = rearrange_corner(mag,cyan,1) 
#transform
transform = perspective_transform(corners[0],corners[1],corners[2],corners[3],imgRGB,500,900)

#on transform maze show walls
wall_contours, transform_maze_RGB = wall_detection(transform) 

#detect robot and orientation
centre, maze_copy_RGB = robot_detector(transform_maze_RGB) 
robotRGB = display(ROBOT_FILE_NAME) 
arUco_corners = arUco_detector(robotRGB) 
direction,maze_copy_RGB = robot_heading(arUco_corners,centre,maze_copy_RGB,corners[4])


#detect target
target = display(IMAGE_LADYBUG_FILE_NAME)

try: #basic colour segmentation
    target_centre = target_detection_2(maze_copy_RGB,direction)
except:#if not enough contours use orb
    target_centre = target_detection(maze_copy_RGB,target,direction)
finally:
    #print and send to text file
    print_walls(maze_copy_RGB,wall_contours,centre,direction,target_centre,MAP_FILE_NAME)

# outputs to webots console and doesn't automatically close
webots = subprocess.Popen(f"{webotsAppLocation} --mode=fast ./worlds/{worldFileName}", encoding='utf8')

# outputs to python terminal and automatically closes
# webots = subprocess.Popen(f"{webotsAppLocation} --mode=fast --stdout --stderr ./worlds/{worldFileName}", 
#                                   stdout = subprocess.PIPE, 
#                                   stderr = subprocess.STDOUT,
#                                   encoding='utf8')

hasControllerFinished = False
while not hasControllerFinished:
    if webots.stdout != None:   
        line = webots.stdout.readline()
        print(line)
        if re.match("INFO: '(.)+' controller exited successfully.", line):
            hasControllerFinished = True
        
        if re.match("WARNING: '(.)+' controller exited with status: (\d)+.", line):
            print(f"\t{RED_TEXT}WARNING: Controller crashed!{END_TEXT_FMT}", flush = True)
            hasControllerFinished = True
time.sleep(2)
webots.kill()
