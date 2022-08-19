import cv2
import matplotlib.pyplot as plt 
import numpy as np

def get_corner_pos(corner):
    M = cv2.moments(corner)
    return (int(M['m10']/M['m00']), int(M['m01']/M['m00']))

def inbounds(corner,bounds):
    if(corner[0] > bounds[0] and corner[0] < bounds[1] and corner[1] > bounds[2] and corner[1] < bounds[3]):
        return True
    return False

def numTostring(direction):
    if direction == 0:
        return "^"
    elif direction == 1:
        return ">"
    elif direction == 2:
        return "v"
    elif direction == 3:
        return "<"

def display(Maze):
    '''
    Displays an image in RGB

    Arguments:
        Maze (.png)  - image of the maze that needs to be displayed in BGR

    Exceptions:
        TypeError  - Occurs when input is not an image of type .png

    Return Value:
        Returns imgRGB - the maze in RGB
    '''
    check=Maze.split(".")
    if check[1]!="png":
        raise TypeError(".png file expected")
    imgBGR = cv2.imread(Maze)
    imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
    plt.imshow(imgRGB)
    plt.show()
    return imgRGB

def detect_corners(imgRGB):
    '''
    Detects the corner stones of the maze and draws them on the maze

    Arguments:
        Maze (.png)  - image of the maze that detection will be performed on

    Exceptions:
        N/A

    Return Value:
        Returns contour_filtered_mag, contour_filtered_cyan - the locations of the cyan and magneta cornerstones
    '''
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
    imgHSV = cv2.cvtColor(imgRGB, cv2.COLOR_BGR2HSV) 
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
    
    #display            
    cv2.drawContours(imgRGB,contour_filtered_cyan, -1, (245, 30, 243), 4)
    cv2.drawContours(imgRGB,contour_filtered_mag, -1, (10, 243, 245), 4)

    plt.imshow(imgRGB)
    plt.show()
       
    return contour_filtered_mag,contour_filtered_cyan

def rearrange_corner(mag,cyan,scale):
    '''
    Allocates corner positions of image to cornerstones

    Arguments:
        mag (list)  - list of all magenta corners
        cyan (list) - list of all cyan corners

    Exceptions:
        ValueError  - Occurs when mag or cyan is not a list of at least length 1 

    Return Value:
        Returns a_corner,b_corner,c_corner,d_corner,inBoxA - locations of all 4 cornerstones and a boolean value 
                                                             representing the location of the pink cornerstone
    '''
    if (len(mag) or len(cyan))<1:
        raise ValueError("Cornerstone lists are too short")
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

def perspective_transform(a_corner,b_corner,c_corner,d_corner,imgRGB,imgHeight,imgWidth):
    '''
    Perfoms a perspective transform of the maze to a new image

    Arguments:
        a_corner (tuple) - tuple of top left corner

        b_corner (tuple) - tuple of top right corner

        c_corner (tuple) - tuple of bottom left corner

        d_corner (tuple) - tuple of bottom right corner

    Exceptions:
        ValueError  - Occurs when ratio of image height to width is not 9:5

    Return Value:
        Returns transform - the transformed image
    '''
    if imgWidth/imgHeight !=1.8:
        raise ValueError("Desired image ratio is not 9:5")
    src = np.float32([a_corner,b_corner,c_corner,d_corner])
    des = np.float32([[0,0],[imgWidth,0],[0,imgHeight],[imgWidth,imgHeight]])
    H_matrix = cv2.getPerspectiveTransform(src,des)
    transform = cv2.warpPerspective(imgRGB, H_matrix, (imgWidth,imgHeight))
    plt.imshow(transform) 
    plt.show() 
    return transform

def wall_detection(transform):
    '''
    Perfoms a wall detection using colour detection

    Arguments:
        transform (.png) - image of the maze after transformation

    Exceptions:
        N/A

    Return Value:
        Returns contour, transform_maze - a list with the detected walls and the annotated maze
    '''
    hMin = 15
    sMin = 0
    vMin = 220
    hMax = 75
    sMax = 255
    vMax = 255

    transform_maze = transform.copy() 
    imgHSV = cv2.cvtColor(transform_maze, cv2.COLOR_RGB2HSV)

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
    
    cv2.drawContours(transform_maze,contour, -1, (0, 0, 245), thickness=cv2.FILLED)
    plt.imshow(transform_maze)
    plt.show()
    
    return contour, transform_maze

def robot_detector(maze):
    '''
    Perfoms robot detection using colour detection

    Arguments:
        maze (.png) - image of the maze after transformation

    Exceptions:
        N/A

    Return Value:
        Returns center ,maze_copy - a tuple with the location of the robot and the annotated maze
    '''
    hMin = 23
    sMin = 45
    vMin = 40
    hMax = 85
    sMax = 175
    vMax = 255
    maze_copy = maze.copy() 
    imgHSV = cv2.cvtColor(maze_copy, cv2.COLOR_RGB2HSV)

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
    
    cv2.circle(maze_copy, center, radius, (255, 0, 0), 2)
    return center ,maze_copy

def arUco_detector(robot):
###############################################################
#The following function to detect the arUco was adapted from:
#Title: Detecting ArUco markers with OpenCV and Python
#Author: Adrian Rosebrock
#Last Updated: December 21, 2020
#Link: https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
###############################################################
    '''
    Uses ArUco library to detect the QR code on the robot

    Arguments:
        robot (.png) - close up image of the robot

    Exceptions:
        N/A

    Return Value:
        Returns corners - list of the positions of each corner of the QR code
    '''
    aruco_get = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    arucoParams =  cv2.aruco.DetectorParameters_create()
    (corners, ids,_) = cv2.aruco.detectMarkers(robot, aruco_get,parameters=arucoParams)
    cv2.aruco.drawDetectedMarkers(robot,corners,ids)
    if (len(corners) > 0):
    # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner,_) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            plt.imshow(robot)
            plt.show()
            return corners

def robot_heading(corners,centre,maze,inAbox):
    '''
    Uses the corner location of the QR code to find the heading of the robot

    Arguments:
        corners (list)    - list of the corner locations of the QR code

        centre  (tuple)   - location of the robot

        maze    (.png)    - image of the trasformed maze

        inAbox  (boolean) - boolean representing the position of the pink cornerstone

    Exceptions:
        N/A

    Return Value:
        Returns direction, maze - int representing the heading of the robot and the annotated maze
    '''
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
    ###############################################################
    #The following function to put text into a circle was adapted from:
    #Title: Draw Text inside circle OpenCV
    #Author: Dan Masek
    #Last Updated: Apr 29, 2019
    #Link: https://stackoverflow.com/questions/55904418/draw-text-inside-circle-opencv
    ###############################################################
    TEXT_FACE = cv2.FONT_HERSHEY_PLAIN 
    TEXT_SCALE = 1.5
    TEXT_THICKNESS = 2
    text_size, _ = cv2.getTextSize(numTostring(direction), TEXT_FACE, TEXT_SCALE, TEXT_THICKNESS)
    text_origin = (int(centre[0] - text_size[0] / 2), int(centre[1] + text_size[1] / 2))

    cv2.putText(maze, numTostring(direction), text_origin, TEXT_FACE, TEXT_SCALE, (255,0,0), TEXT_THICKNESS, cv2.LINE_AA)
    plt.imshow(maze)
    plt.show()
    return direction,maze

def target_detection(maze,target,direction):
    '''
    Uses the orb detection to find the target

    Arguments:
        maze (.png)     - image of the trasformed maze

        target (.png)   - image of the desired target

        direction (int) - int representing the heading of the robot

    Exceptions:
        N/A

    Return Value:
        Returns (mp[0], mp[1]) - tuple with the location of the target
    '''
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

    # Initialize the Matcher for matching the keypoints and then match the keypoints
   
    matcher = cv2.BFMatcher()
    matches = matcher.knnMatch(mazeDescriptors,targetDescriptors,k=2)

    viable_match = []
    for m, n in matches: #lowes ratio
        if(m.distance < 0.65*n.distance): 
            viable_match.append(m) 
    
    img3 = cv2.drawMatches(maze_img,mazeKeypoints,target_img,targetKeypoints,viable_match,None,flags = 2)
    plt.imshow(img3)
    plt.show()
    
    src_pts = np.float32([ targetKeypoints[m.trainIdx].pt for m in viable_match ]).reshape(-1,1,2)
    dst_pts = np.float32([ mazeKeypoints[m.queryIdx].pt for m in viable_match ]).reshape(-1,1,2)
    M,_ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    h,w = target_img.shape
    mp = cv2.perspectiveTransform(np.float32([[w/2.0, h/2.0]]).reshape(-1,1,2), M)[0][0]
    maze = cv2.circle(maze, (mp[0], mp[1]), 20, (0, 255, 0), 4)
    
    TEXT_FACE = cv2.FONT_HERSHEY_PLAIN 
    TEXT_SCALE = 1.5
    TEXT_THICKNESS = 2
    text_size, _ = cv2.getTextSize(numTostring(direction), TEXT_FACE, TEXT_SCALE, TEXT_THICKNESS)
    text_origin = (int(mp[0] - text_size[0] / 2), int(mp[1] + text_size[1] / 2))

    maze = cv2.putText(maze, "X", text_origin, TEXT_FACE, TEXT_SCALE, (0,255,0), TEXT_THICKNESS, cv2.LINE_AA)
    
    plt.imshow(maze)
    plt.show()
    return(mp[0], mp[1])

def target_detection_2(maze,direction):
    '''
    Uses the colour detection detection to find the target

    Arguments:
        maze (.png)     - image of the trasformed maze

        direction (int) - int representing the heading of the robot

    Exceptions:
        N/A

    Return Value:
        Returns centre - tuple with the location of the target
    '''
    img_hsv = cv2.cvtColor(maze, cv2.COLOR_RGB2HSV) 
    lower_red = np.array([170,144,144])
    upper_red = np.array([180,255,255])
    mask = cv2.inRange(img_hsv, lower_red, upper_red)

    kernel = np.ones((7,7),np.uint8)
    closing = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours,_ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
                               
    if (cv2.contourArea(contours[0]) < cv2.contourArea(contours[1])):
        centre = get_corner_pos(contours[0])
    else:
        centre = get_corner_pos(contours[1])
        
    maze = cv2.circle(maze, (centre[0], centre[1]), 20, (0, 255, 0), 4)
    TEXT_FACE = cv2.FONT_HERSHEY_PLAIN 
    TEXT_SCALE = 1.5
    TEXT_THICKNESS = 2
    text_size, _ = cv2.getTextSize(numTostring(direction), TEXT_FACE, TEXT_SCALE, TEXT_THICKNESS)
    text_origin = (int(centre[0] - text_size[0] / 2), int(centre[1] + text_size[1] / 2))

    maze = cv2.putText(maze, "X", text_origin, TEXT_FACE, TEXT_SCALE, (0,255,0), TEXT_THICKNESS, cv2.LINE_AA)
    plt.imshow(maze)
    plt.show()
    return centre

def print_walls(maze,wall_contours,robot_centre,direction,target_centre,file_name):
    '''
    Uses the colour detection detection to find the target

    Arguments:
        maze (.png)           - image of the trasformed maze

        wall_contours (list)  - list with the contours of the walls

        robot_centre (tuple)  - location of the robot

        direction (int)       - int representing the heading of the robot

        target_centre (tuple) - location of the target

        file_name (string)    - name of the output file

    Exceptions:
        N/A

    Return Value:
        N/A
    '''
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
        
        if (inbounds(robot_centre,bounds) and bounds[0] % 100 == 0 and bounds[1] % 100 == 0 ):
            vert_walls.append(" " + numTostring(direction) + " ")
        elif (inbounds(target_centre,bounds) and bounds[0] % 100 == 0 and bounds[1] % 100 == 0 ):
            vert_walls.append(" x ")
        elif (bounds[0] % 100 == 0 and bounds[1] % 100 == 0 ):
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
        if( p == "|\n"):
            while i < 80:
                maze = maze + hori_walls[i]
                if (hori_walls[i] == "\n"):
                    i = i + 1
                    break
                i = i + 1

    maze =  maze + " --- --- --- --- --- --- --- --- --- \n"   
    print(maze)
    f = open(file_name, "w")
    f.write(maze)
    f.close()
    return 