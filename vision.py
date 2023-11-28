'''
vision.py
vision
Authors: Benoît Gallois, Jehan Corcelle, Arto Dubuisson, Raphaël Dousson
'''

import cv2
import time
import numpy as np


# Definition of the colours thresholds
LOWER_RED = np.array([0, 100, 100])
UPPER_RED = np.array([10, 255, 255])

LOWER_BLUE = np.array([100, 100, 100])
UPPER_BLUE = np.array([140, 255, 255])

LOWER_GREEN = np.array([40, 40, 40])
UPPER_GREEN = np.array([80, 255, 255])

LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([179, 255, 30])

# Definition of the size of contours considered as noise
NOISY_CONTOUR_LENGHT = 2000

MARGIN_RED_BLUE_GREEN = 0  
MARGIN_OBSTACLE = 1200

def detect_area(image, lower_colour, upper_colour, margin):
    '''
    @brief   Detects areas corresponding to a color and returns the coordinates of the vertices of these areas.

    @param   image        -> Image array (numpy array) captured from the camera
             lower_colour -> LOWER_RED, LOWER_BLACK, LOWER_BLUE, LOWER_GREEN
             upper_colour -> UPPER_RED, UPPER_BLACK, UPPER_BLUE, UPPER_GREEN
             margin       -> MARGIN_OBSTACLE, MARGIN_RED_BLUE_GREEN

    @return  coords       -> list of the coordinates of the vertices for each area
    '''

    height, width, _ = image.shape  # Give the size of the image

    # Converts the image in the HSV space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Filter the image to retain only pixels of the desired color
    mask = cv2.inRange(hsv, lower_colour, upper_colour)

    # Blur masks to reduce noise
    blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # Find contours in the filtered mask
    contours, _ = cv2.findContours(blurred_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # List to store coordinates of detected zones
    coords = []

    # Browse all contours found
    for contour in contours:
        
        # Ignore small contours that could be noise
        if cv2.contourArea(contour) > NOISY_CONTOUR_LENGHT:
            
            # Get the coordinates of the rectangle enclosing the area
            x, y, w, h = cv2.boundingRect(contour)
            
            # Add the coordinates of the zone's vertices with a change of reference point (bottom left corner)
            # Ajoutez la marge de 300 pixels
            x_with_margin = max(0, x - margin)
            y_with_margin = max(0, y - margin)
            w_with_margin = min(width, w + 2 * margin)
            h_with_margin = min(height, h + 2 * margin)

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
    
    # Calculate the centroid of the front area (blue)
    front_centroid = np.mean(np.array(front_area).reshape(-1, 2), axis=0)
    
    # Calculate the centroid of the back area (green)
    back_centroid = np.mean(np.array(back_area).reshape(-1, 2), axis=0)
    
    # Calculate the direction vector of the robot
    direction = (front_centroid - back_centroid).astype(int)
    
    # Calculate the midpoint between the centroids
    midpoint = ((front_centroid + back_centroid) / 2).astype(int)
    
    return tuple(direction), midpoint



def calculate_area_centroid(area_coordinates):
    '''
    @brief   Calculates the centroid coordinates of an area.
 
    @param   area_coordinates -> List of coordinates area
    
    @return  centroid         -> List that represents the centroid coordinates
    '''
    
    # Calculate the centroid of the area
    centroid = np.mean(np.array(area_coordinates).reshape(-1, 2), axis=0)
    
    return centroid

def vision_obstacles_positions():
    # Ouvrir la webcam (index 0 par défaut)
    cap = cv2.VideoCapture(0)

    # Vérifier si la webcam est ouverte correctement
    if not cap.isOpened():
        print("Erreur: Impossible d'ouvrir la webcam.")
        return

    try:
        while True:
            # Capturer une image depuis la webcam
            ret, frame = cap.read()

            if not ret:
                print("Erreur: Impossible de capturer l'image.")
                break

            # Afficher l'image
            cv2.imshow('Webcam', frame)

            # Find the contours of each areas
            obstacles = detect_area(frame, LOWER_BLACK, UPPER_BLACK, MARGIN_OBSTACLE)
            goal_area = detect_area(frame, LOWER_RED, UPPER_RED, MARGIN_RED_BLUE_GREEN)
            front_robot_area = detect_area(frame, LOWER_BLUE, UPPER_BLUE, MARGIN_RED_BLUE_GREEN)
            back_robot_area = detect_area(frame, LOWER_GREEN, UPPER_GREEN, MARGIN_RED_BLUE_GREEN)

            # Find the robot direction vector and its midpoint
            robot_direction = calculate_robot_direction(front_robot_area, back_robot_area)
            robot_centroid = calculate_area_centroid(back_robot_area)

            # Find the centroid of the goal area
            goal_centroid = calculate_area_centroid(goal_area)
            

            # Afficher les coordonnées détectées (vous pouvez les utiliser comme nécessaire)
            print(f"Coordonnées obstacles noirs :", obstacles)
            print("Vecteur direction du robot :", robot_direction[0])
            print("Centroid du robot :", robot_centroid)
            print("Centroid de la goal area :", goal_centroid)

            # Attendre 3 secondes (3000 millisecondes)
            time.sleep(3)

            # Quitter si la touche 'q' est enfoncée
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Libérer la webcam et fermer la fenêtre
        cap.release()
        cv2.destroyAllWindows()
    
    return obstacles, goal_centroid, robot_centroid, robot_direction



def vision_robot():
    # Ouvrir la webcam (index 0 par défaut)
    cap = cv2.VideoCapture(0)

    # Vérifier si la webcam est ouverte correctement
    if not cap.isOpened():
        print("Erreur: Impossible d'ouvrir la webcam.")
        return

    try:
        while True:
            # Capturer une image depuis la webcam
            ret, frame = cap.read()

            if not ret:
                print("Erreur: Impossible de capturer l'image.")
                break

            # Afficher l'image
            cv2.imshow('Webcam', frame)

            # Find the contours of each areas
            front_robot_area = detect_area(frame, LOWER_BLUE, UPPER_BLUE, MARGIN_RED_BLUE_GREEN)
            back_robot_area = detect_area(frame, LOWER_GREEN, UPPER_GREEN, MARGIN_RED_BLUE_GREEN)

            # Find the robot direction vector and its midpoint
            robot_direction = calculate_robot_direction(front_robot_area, back_robot_area)
            robot_centroid = calculate_area_centroid(back_robot_area)
            

            # Afficher les coordonnées détectées (vous pouvez les utiliser comme nécessaire)
            print("Vecteur direction du robot :", robot_direction[0])
            print("Centroid du robot :", robot_centroid)

            # Attendre 3 secondes (3000 millisecondes)
            time.sleep(3)

            # Quitter si la touche 'q' est enfoncée
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Libérer la webcam et fermer la fenêtre
        cap.release()
        cv2.destroyAllWindows()
    
    return robot_centroid, robot_direction

# Appeler la fonction principale
# vision_obstacles_positions()