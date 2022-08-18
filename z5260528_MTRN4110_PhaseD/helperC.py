import cv2
import numpy as np

def display_maze(Maze):
    imgBGR = cv2.imread(Maze)
    return imgBGR

def detect_corners(imgBGR):
    hMin_cyan = 30
    sMin_cyan = 115
    vMin_cyan = 226
    hMax_cyan = 95
    sMax_cyan = 255
    vMax_cyan = 255
    hMin_mag = 130
    sMin_mag = 56
    vMin_mag = 242
    hMax_mag = 150
    sMax_mag = 255
    vMax_mag = 255

    #cyan corner first
    imgHSV = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV) 
    lower = np.array([hMin_cyan, sMin_cyan, vMin_cyan])
    upper = np.array([hMax_cyan, sMax_cyan, vMax_cyan])
    mask = cv2.inRange(imgHSV, lower, upper)
    kernel = np.ones((10,10), np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours_cyan,_ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    contour_filtered_cyan = []
    for _,c in enumerate(contours_cyan):
            area = cv2.contourArea(c)
            if (area > 50):
                contour_filtered_cyan.append(c)
    
    #magenta corner
    lower = np.array([hMin_mag, sMin_mag, vMin_mag])
    upper = np.array([hMax_mag, sMax_mag, vMax_mag])
    mask = cv2.inRange(imgHSV, lower, upper)
    opening = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
   
    contours_magenta,_ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    contour_filtered_mag = []
    for _,c in enumerate(contours_magenta):
            area = cv2.contourArea(c)
            if (area > 50):
                contour_filtered_mag.append(c)        
     
    return contour_filtered_mag,contour_filtered_cyan

def get_corner_pos(corner):
    M = cv2.moments(corner)
    return (int(M['m10']/M['m00']), int(M['m01']/M['m00']))

def inbounds(corner,bounds):
    if(corner[0] > bounds[0] and corner[0] < bounds[1] and corner[1] > bounds[2] and corner[1] < bounds[3]):
        return True
    return False

def rearrange_corner(mag,cyan,scale):
    #[xlower,xupper,ylower,yupper]
    BoxA_bounds = [0,337.5/scale,0,375]
    BoxB_bounds = [1012.5/scale,1350/scale,0,375] 
    BoxC_bounds = [0,337.5/scale,375,750] 
    BoxD_bounds = [1012.5/scale,1350/scale,375,750] 

    #find magenta
    d_corner = get_corner_pos(mag[0])
    #check if in box D
    if (inbounds(d_corner,BoxD_bounds)):
        inBoxA = False
        for c in cyan:
            corner = get_corner_pos(c)
            if (inbounds(corner,BoxA_bounds)):
                a_corner = corner
            if (inbounds(corner,BoxB_bounds)):
                b_corner = corner
            if (inbounds(corner,BoxC_bounds)):
                c_corner = corner
    #check if in box A        
    elif (inbounds(d_corner,BoxA_bounds)):
        inBoxA = True
        for c in cyan:
            corner = get_corner_pos(c)
            if (inbounds(corner,BoxD_bounds)):
                a_corner = corner
            if (inbounds(corner,BoxB_bounds)):
                c_corner = corner
            if (inbounds(corner,BoxC_bounds)):
                b_corner = corner     
                
    return a_corner,b_corner,c_corner,d_corner,inBoxA

def numTostring(direction):
    if direction is 0:
        return "^"
    elif direction is 1:
        return ">"
    elif direction is 2:
        return "v"
    elif direction is 3:
        return "<"

def perspective_transform(a_corner,b_corner,c_corner,d_corner,imgBGR,imgHeight,imgWidth):
    src = np.float32([a_corner,b_corner,c_corner,d_corner])
    des = np.float32([[0,0],[imgWidth,0],[0,imgHeight],[imgWidth,imgHeight]])
    H_matrix = cv2.getPerspectiveTransform(src,des)
    transform = cv2.warpPerspective(imgBGR, H_matrix, (imgWidth,imgHeight))
    return transform

def wall_detection(transform):
    hMin = 15
    sMin = 0
    vMin = 220
    hMax = 75
    sMax = 255
    vMax = 255

    transform_maze = transform.copy() 
    imgHSV = cv2.cvtColor(transform_maze, cv2.COLOR_BGR2HSV)

    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])
    mask = cv2.inRange(imgHSV, lower, upper)

    kernel = np.ones((3,3), np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours,_ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    contour = []
    for _,c in enumerate(contours):
            area = cv2.contourArea(c)
            if (area > 100):
                contour.append(c)          
    
    return contour, transform_maze

def robot_detector(maze):
    hMin = 23
    sMin = 45
    vMin = 40
    hMax = 85
    sMax = 175
    vMax = 255
    maze_copy = maze.copy() 
    imgHSV = cv2.cvtColor(maze_copy, cv2.COLOR_BGR2HSV)

    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])
    mask = cv2.inRange(imgHSV, lower, upper)

    kernel = np.ones((5,5), np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours,_ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    contour = []
    for _,c in enumerate(contours):
            area = cv2.contourArea(c)
            if (area > 50):
                contour.append(c)
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)
                
    return center, radius ,maze_copy


def arUco_detector(robot):
###############################################################
#The following function to detect the arUco was adapted from:
#Title: Detecting ArUco markers with OpenCV and Python
#Author: Adrian Rosebrock
#Last Updated: December 21, 2020
#Link: https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
###############################################################
    
    ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }
    for aruco in ARUCO_DICT.items():
        aruco_get = cv2.aruco.Dictionary_get(aruco[1])
        arucoParams =  cv2.aruco.DetectorParameters_create()
        (corners, ids,_) = cv2.aruco.detectMarkers(robot, aruco_get,parameters=arucoParams)
        if (len(corners) > 0):
        # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner,_) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                return corners

def robot_heading(corners,inAbox):
    corner_heading = corners[0]
    opp_corner = corners[2]
    #check if its on the y axis or x axis
    if (abs(corner_heading[0] - opp_corner[0]) > abs(corner_heading[1] - opp_corner[1])):
        if (corner_heading[0] > opp_corner[0]):
            direction = 1
        else:
            direction = 3
    else:
        if (corner_heading[1] > opp_corner[1]):
            direction = 2
        else:
            direction = 0
    if (inAbox):
        direction = (direction + 2) % 4

    return direction

def target_detection(maze,target):
    ###############################################################
    #The following function to detect features using orb and lowe's ratio was adapted from:
    #Title: Some simple template matching with ORB
    #Author: fehlfarbe
    #Last Updated: 
    #Link:https://gist.github.com/fehlfarbe/a2a9058e05f364d31239a425ffddcb2e
    ###############################################################
    maze_img = cv2.cvtColor(maze,cv2.COLOR_BGR2GRAY)
    target_img = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(nfeatures = 20000)#high sample 
    
    targetKeypoints, targetDescriptors = orb.detectAndCompute(target_img,None)        
    mazeKeypoints, mazeDescriptors = orb.detectAndCompute(maze_img,None)

    # Initialize the Matcher for matching
    # the keypoints and then match the
    # keypoints


    matcher = cv2.BFMatcher()
    matches = matcher.knnMatch(mazeDescriptors,targetDescriptors,k=2)

    viable_match = []
    for m, n in matches: #lowes ratio
        if(m.distance < 0.65*n.distance): 
            viable_match.append(m) 

    src_pts = np.float32([ targetKeypoints[m.trainIdx].pt for m in viable_match ]).reshape(-1,1,2)
    dst_pts = np.float32([ mazeKeypoints[m.queryIdx].pt for m in viable_match ]).reshape(-1,1,2)
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    # matchesMask = mask.ravel().tolist()
    h,w = target_img.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    # dst = cv2.perspectiveTransform(pts,M)
    mp = cv2.perspectiveTransform(np.float32([[w/2.0, h/2.0]]).reshape(-1,1,2), M)[0][0]

    return(mp[0], mp[1])

def target_detection_2(maze):
    img_hsv = cv2.cvtColor(maze, cv2.COLOR_BGR2HSV) 
    lower_red = np.array([170,144,144])
    upper_red = np.array([180,255,255])
    mask = cv2.inRange(img_hsv, lower_red, upper_red)

    kernel = np.ones((7,7),np.uint8)
    closing = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours,_ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]                      
    c = []

    for _,n in enumerate(contours):
        c.append(n)
    if (cv2.contourArea(c[0]) < cv2.contourArea(c[1])):
        centre = get_corner_pos(c[0])
    else:
        centre = get_corner_pos(c[1])

    return centre

def print_walls(maze,wall_contours,robot_centre,direction,target_centre,file_name):
    #use colour and mask
    vert_walls = []
    hori_walls = []
    wall_centres = []
    for c in wall_contours:
        wall_centres.append(get_corner_pos(c))
    #check vertical walls
    i = 0
    bounds = [0,100,0,100]
    vert_walls.append("|")
    while (i < 5):
        
        if (inbounds(robot_centre,bounds) and bounds[0] % 100 is 0 and bounds[1] % 100 is 0 ):
            vert_walls.append(" " + numTostring(direction) + " ")
        elif (inbounds(target_centre,bounds) and bounds[0] % 100 is 0 and bounds[1] % 100 is 0 ):
            vert_walls.append(" x ")
        elif (bounds[0] % 100 is 0 and bounds[1] % 100 is 0 ):
            vert_walls.append("   ")
        else:
            wall_detected = False
            for c in wall_centres:
                if(inbounds(c,(bounds[0],bounds[1],bounds[2]+10,bounds[3]-10))):
                    wall_centres.remove(c)
                    wall_detected = True
                    vert_walls.append("|")
                    break
            if (not wall_detected):
                vert_walls.append(" ")
        bounds[0] = bounds[0] + 50
        bounds[1] = bounds[1] + 50
        if (bounds[1] > 900):
            bounds[0] = 0
            bounds[1] = 100
            bounds[2] = bounds[2] + 100
            bounds[3] = bounds[3] + 100
            vert_walls.append("|\n")
            i = i + 1
            if (i < 5):
                vert_walls.append("|")
    
    
    #check horizontal walls
    i = 0
    bounds = [0,100,50,150]
    hori_walls.append(" ")
    while (i < 4):
        wall_detected = False
        for c in wall_centres:
            if(inbounds(c,(bounds[0]+10,bounds[1]-10,bounds[2],bounds[3]))):
                wall_centres.remove(c)
                wall_detected = True
                hori_walls.append("---")
                break
        if (not wall_detected):
            hori_walls.append("   ")
        
        bounds[0] = bounds[0] + 100
        bounds[1] = bounds[1] + 100
        hori_walls.append(" ")
        if (bounds[1] > 900):
            bounds[0] = 0
            bounds[1] = 100
            bounds[2] = bounds[2] + 100
            bounds[3] = bounds[3] + 100
            hori_walls.append("\n")
            i = i + 1
            if (i < 4):
                hori_walls.append(" ")
    
    
    
    i = 0
    maze =  " --- --- --- --- --- --- --- --- --- \n"
    for p in vert_walls:
        maze = maze + p
        if( p is "|\n"):
            while i < 80:
                maze = maze + hori_walls[i]
                if (hori_walls[i] is "\n"):
                    i = i + 1
                    break
                i = i + 1

    maze =  maze + " --- --- --- --- --- --- --- --- --- \n"   
    print(maze)
    f = open(file_name, "w")
    f.write(maze)
    f.close()
    return 