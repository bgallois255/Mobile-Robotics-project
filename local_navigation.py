'''
File : local_navigation.py 
Author : Jehan Corcelle
Date : 6 déc 2023
Detect inapropriate obstcles and avoid them.
'''

#==================================================================
import numpy as np
import matplotlib.pyplot as plt
#==================================================================

# Constants determined by experiments
THRESHOLD = 20
CONSTANT_A = 5739
CONSTANT_B = 339

# Angle of the different proximity sensors
ANGLE_CAPTEUR_1 = 40
ANGLE_CAPTEUR_2 = 25
ANGLE_CAPTEUR_3 = 0

# Margin size around the obstacle
MARGIN_X = 10
MARGIN_Y = 12

#==================================================================

def object_robot_coordinates(prox_sensors):
    '''
    @brief   Returns the position of the detected object in the robot reference frame.
 
    @param   prox_sensors        -> List representing the values reported by the proximity sensors
    
    @return  object_robot_coords -> List representing the coordinates of the detected object or None if no object is detected
    '''
    
    # Check if any sensor detected an object
    if any(sensor_value > THRESHOLD for sensor_value in prox_sensors):
        # Find the index and value of the sensor with the maximum value
        max_sensor_index, max_sensor_value = max(enumerate(prox_sensors), key=lambda x: x[1]) 

        # Calculate distance and angle based on the sensor index
        distance = (CONSTANT_A - max_sensor_value) / CONSTANT_B
        angle = [ANGLE_CAPTEUR_1, ANGLE_CAPTEUR_2, ANGLE_CAPTEUR_3, -ANGLE_CAPTEUR_2, -ANGLE_CAPTEUR_1][max_sensor_index]
        
        # Calculate object coordinates in the robot's reference frame
        object_robot_coords = [distance * np.cos(np.radians(angle)),
                               distance * np.sin(np.radians(angle))]

        return object_robot_coords
    
    # Return empty list if no object is detected
    return []


def robot_to_ground_coords(robot_centroid, robot_direction, object_robot_coords):
    '''
    @brief   Converts the detected object coordinates from the robot's reference frame to the ground reference frame.
 
    @param   robot_centroid        -> List representing the centroid coordinates of the robot
             robot_direction       -> Tuple representing the direction vector coordinates of the robot
             object_robot_coords   -> List representing the coordinates of the detected object in the robot's reference frame
    
    @return  object_ground_coords  -> List representing the coordinates of the detected object in the ground reference frame
    '''
    
    # Convert robot direction to unit vector
    robot_direction_unit = robot_direction / np.linalg.norm(robot_direction)
    
    # Calculate the vector from the robot centroid to the detected object in the robot's reference frame
    object_vector_robot_frame = np.array(object_robot_coords)
    
    # Transform the vector to the ground reference frame
    object_vector_ground_frame = object_vector_robot_frame.dot([[robot_direction_unit[0], robot_direction_unit[1]], 
                                                                [- robot_direction_unit[1], robot_direction_unit[0]]])
    
    # Calculate the coordinates of the detected object in the ground reference frame
    object_ground_coords = robot_centroid + object_vector_ground_frame
    
    return object_ground_coords



def safety_zone(center_point):
    '''
    @brief   Creates four corners defining a safety zone around a given center point.
 
    @param   center_point                           -> Tuple representing the coordinates of the center point
    
    @return  [corner1, corner2, corner3, corner4]   -> Four corners that defined the safety zone
    '''
    
    # Calculate coordinates of the four corners
    corner1 = (center_point[0] + MARGIN_X, center_point[1] + MARGIN_Y)
    corner2 = (center_point[0] + MARGIN_X, center_point[1] - MARGIN_Y)
    corner3 = (center_point[0] - MARGIN_X, center_point[1] - MARGIN_Y)
    corner4 = (center_point[0] - MARGIN_X, center_point[1] + MARGIN_Y)
    
    return [corner1, corner2, corner3, corner4]


def is_inside(point, obstacles):
    '''
    @brief   When it detects an obstacle, it checks if it has not been already detected

    @param   point           -> List of the coordinates of the center of the new obstacle
             obstacles       -> List of coordinates of the obstacles on the map
             
    @return  boolean
    '''
    for obstacle in range(len(obstacles)):
        min_x = min(s[0] for s in obstacles[obstacle])
        max_x = max(s[0] for s in obstacles[obstacle])
        min_y = min(s[1] for s in obstacles[obstacle])
        max_y = max(s[1] for s in obstacles[obstacle])
        for summit in range(len(obstacles[obstacle])):
            if min_x <= point[0] <= max_x and min_y <= point[1] <= max_y:
                return True
    return False

