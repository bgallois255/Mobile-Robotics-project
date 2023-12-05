'''
vision.py
vision
Authors: Benoît Gallois, Jehan Corcelle, Arto Dubuisson, Raphaël Dousson
'''

import cv2
import time
import numpy as np

# Definition of the colours thresholds

LOWER_RED = np.array([0, 100, 50])
UPPER_RED = np.array([15, 255, 255])

LOWER_BLUE = np.array([100, 100, 100])
UPPER_BLUE = np.array([140, 255, 255])

LOWER_GREEN = np.array([40, 40, 40])
UPPER_GREEN = np.array([80, 255, 255])

LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([179, 255, 30])

# Definition of the size of contours considered as noise
NOISY_CONTOUR_LENGHT = 2000

MARGIN_RED_BLUE_GREEN = 0
MARGIN_OBSTACLE = 0


def detect_area(image, lower_colour, upper_colour, margin):
    height, width, _ = image.shape

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_colour, upper_colour)

    blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)

    contours, _ = cv2.findContours(blurred_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    coords = []

    for contour in contours:
        if cv2.contourArea(contour) > NOISY_CONTOUR_LENGHT:
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
    front_centroid = np.mean(np.array(front_area).reshape(-1, 2), axis=0)
    back_centroid = np.mean(np.array(back_area).reshape(-1, 2), axis=0)
    direction = (front_centroid - back_centroid).astype(int)
    midpoint = ((front_centroid + back_centroid) / 2).astype(int)
    return tuple(direction), midpoint


def calculate_area_centroid(area_coordinates):
    centroid = np.mean(np.array(area_coordinates).reshape(-1, 2), axis=0)
    return centroid


def vision_obstacles_and_goal():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Erreur: Impossible d'ouvrir la webcam.")
        return

    time.sleep(1)
    ret, frame = cap.read()
    
    if not ret:
        print("Erreur: Impossible de capturer l'image.")
        return

    cv2.imshow('Webcam', frame)

    obstacles = detect_area(frame, LOWER_BLACK, UPPER_BLACK, MARGIN_OBSTACLE)
    goal_area = detect_area(frame, LOWER_RED, UPPER_RED, MARGIN_RED_BLUE_GREEN)
   
    goal_centroid = calculate_area_centroid(goal_area)

    print(f"Coordonnées obstacles noirs :", obstacles)
    print("Centroid de la goal area :", goal_centroid)
    
    return obstacles, goal_centroid


def vision_robot():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Erreur: Impossible d'ouvrir la webcam.")
        return

    time.sleep(1)
    ret, frame = cap.read()
    
    if not ret:
        print("Erreur: Impossible de capturer l'image.")
        return

    cv2.imshow('Webcam', frame)

    front_robot_area = detect_area(frame, LOWER_BLUE, UPPER_BLUE, MARGIN_RED_BLUE_GREEN)
    back_robot_area = detect_area(frame, LOWER_GREEN, UPPER_GREEN, MARGIN_RED_BLUE_GREEN)

    robot_direction, robot_centroid = calculate_robot_direction(front_robot_area, back_robot_area)

    print("Vecteur direction du robot :", robot_direction[0])
    print("Centroid du robot :", robot_centroid)
    
    return robot_centroid, robot_direction