'''
File : vision.py 
Author : Benoît Gallois
Date : 6 déc 2023
Detect the map: obstacles, goal, robot positions.
'''

#==================================================================
import cv2
import time
import numpy as np
#==================================================================

# Color thresholds
LOWER_RED = np.array([0, 100, 50])
UPPER_RED = np.array([15, 255, 255])

LOWER_BLUE = np.array([100, 100, 100])
UPPER_BLUE = np.array([140, 255, 255])

LOWER_GREEN = np.array([40, 40, 40])
UPPER_GREEN = np.array([80, 255, 255])

LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([179, 255, 30])

# Maximum size of contours considered as noise
NOISY_CONTOUR_LENGHT = 2000

# Margin sizes
MARGIN_RED_BLUE_GREEN = 0
MARGIN_OBSTACLE = 140

#==================================================================

def detect_area(image, lower_color, upper_color, margin):
    '''
    @brief   Detects areas corresponding to a color and returns the coordinates of the vertices and add a margin.

    @param   image        -> Image array (numpy array) captured from the camera
             lower_color -> LOWER_RED, LOWER_BLACK, LOWER_BLUE, LOWER_GREEN
             upper_color -> UPPER_RED, UPPER_BLACK, UPPER_BLUE, UPPER_GREEN
             margin       -> MARGIN_OBSTACLE, MARGIN_RED_BLUE_GREEN

    @return  coords       -> list of the coordinates of the vertices for each area
    '''
    
    height, width, _ = image.shape

    # Convert image in HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Create a mask that detect only the selected color
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Remove noise
    blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)

    contours, _ = cv2.findContours(blurred_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    coords = []

    for contour in contours:
        if cv2.contourArea(contour) > NOISY_CONTOUR_LENGHT:    # If the detected area is too small with consider it as noise
            x, y, w, h = cv2.boundingRect(contour)
            x_with_margin = max(0, x - margin)
            y_with_margin = max(0, y - margin)
            w_with_margin = min(width - x_with_margin, w + 2 * margin)
            h_with_margin = min(height - y_with_margin, h + 2 * margin)

            coords.append([(x_with_margin, height - y_with_margin),
                           (x_with_margin + w_with_margin, height - y_with_margin),
                           (x_with_margin + w_with_margin, height - y_with_margin - h_with_margin),
                           (x_with_margin, height - y_with_margin - h_with_margin)])

    return coords


def calculate_robot_direction(front_area, back_area):
    '''
    @brief   Calculates the direction vector of the robot.
 
    @param   front_area   -> List of coordinates for the front area (blue)
             back_area    -> List of coordinates for the back area (green)
    
    @return  direction    -> tuple represents the direction vector coordinates
                             list represents the midpoint coordinates
    '''
    
    front_centroid = np.mean(np.array(front_area).reshape(-1, 2), axis=0)
    back_centroid = np.mean(np.array(back_area).reshape(-1, 2), axis=0)
    
    direction = (front_centroid - back_centroid).astype(int)
    midpoint = ((front_centroid + back_centroid) / 2).astype(int)
    
    return tuple(direction), midpoint


def calculate_area_centroid(area_coordinates):
    '''
    @brief   Calculates the centroid coordinates of an area.
 
    @param   area_coordinates -> List of coordinates area
    
    @return  centroid         -> List that represents the centroid coordinates
    '''
    
    centroid = np.mean(np.array(area_coordinates).reshape(-1, 2), axis=0)
    
    return centroid


def vision_obstacles_and_goal(frame):
    '''
    @brief   Open the camera and compute the obstacles and the goal coordinates.
    
    @param   frame           -> Image capture from the webcam
    
    @return  obstacles       -> List that represents the vertices of the obstacles coordinates
             goal_centroid   -> List of coordinates of the goal centroid
    '''

    obstacles = detect_area(frame, LOWER_BLACK, UPPER_BLACK, MARGIN_OBSTACLE)
    goal_area = detect_area(frame, LOWER_RED, UPPER_RED, MARGIN_RED_BLUE_GREEN)
   
    goal_centroid = calculate_area_centroid(goal_area)

    print(f"Coordonnées obstacles noirs :", obstacles)
    print("Centroid de la goal area :", goal_centroid)
    
    return obstacles, goal_centroid


def vision_robot(frame):
    '''
    @brief   Open the camera and compute the robot coordinates and direction.
    
    @param   frame             -> Image capture from the webcam
    
    @return  robot_centroid    -> List of the coordinates of the robot centroid
             robot_direction   -> Tuple of the robot direction (x,y)
    '''
    
    front_robot_area = detect_area(frame, LOWER_BLUE, UPPER_BLUE, MARGIN_RED_BLUE_GREEN)
    back_robot_area = detect_area(frame, LOWER_GREEN, UPPER_GREEN, MARGIN_RED_BLUE_GREEN)

    robot_direction, robot_centroid = calculate_robot_direction(front_robot_area, back_robot_area)

    print("Vecteur direction du robot :", robot_direction[0], robot_direction[1])
    print("Centroid du robot :", robot_centroid)
    
    return robot_centroid, robot_direction


def converter(coordinates, ratio):
    '''
    @brief   Converts coordinates to a given ratio.
 
    @param   coordinates             -> List of coordinates 
             ratio                   -> PIXEL_TO_CM_RATIO or CM_TO_PIXEL_RATIO
    
    @return  converted_coordinates   -> List of the converted coordinates
    '''
    
    # Convert each coordinate in the list or individual point
    if isinstance(coordinates[0], (list, tuple)):
        # List of coordinates
        converted_coordinates = [(int(x * ratio), int(y * ratio)) for x, y in coordinates]
    
    else:
        # Single coordinate
        converted_coordinates = (int(coordinates[0] * ratio), int(coordinates[1] * ratio))
    
    return converted_coordinates