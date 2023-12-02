import numpy as np
import matplotlib.pyplot as plt

def object_robot_coordinates(prox_sensors):
    '''
    @brief   Returns the position of the detected object in the robot reference frame.
 
    @param   robot_centroid      -> List representing the centroid coordinates of the robot
             robot_direction    -> Tuple representing the direction vector coordinates of the robot
             prox_sensors       -> List representing the values reported by the proximity sensors
    
    @return  object_robot_coords -> List representing the coordinates of the detected object in the robot reference frame, or None if no object is detected
             max_sensor_index   -> Index of the sensor with the maximum value among those that detected an object
             max_sensor_value   -> Maximum value among the sensors that detected an object
    '''
    
    # Check if any sensor detected an object
    if any(sensor_value > 20 for sensor_value in prox_sensors):
        # Find the index and value of the sensor with the maximum value
        max_sensor_index, max_sensor_value = max(enumerate(prox_sensors), key=lambda x: x[1]) 
        '''
        max(..., key=lambda x: x[1]) spécifie une fonction lambda en tant que clé de comparaison pour la fonction max. 
        lambda prend une paire (indice, valeur) et retourne la valeur (x[1]) comme critère de comparaison.
        '''     
        # Calculate distance and angle based on the sensor index
        distance = (5739 - max_sensor_value) / 339
        angle = [40, 25, 0, -25, -40][max_sensor_index]
        
        # Calculate object coordinates in the robot's reference frame
        object_robot_coords = [
            distance * np.cos(np.radians(angle)),
            distance * np.sin(np.radians(angle))
        ]

        return object_robot_coords
    
    # Return empty list if no object is detected
    return []




def robot_to_ground_coords(robot_centroid, robot_direction, object_robot_coords):
    '''
    @brief   Converts the detected object coordinates from the robot's reference frame to the ground reference frame.
 
    @param   robot_centroid         -> List representing the centroid coordinates of the robot
             robot_direction       -> Tuple representing the direction vector coordinates of the robot
             object_robot_coords -> List representing the coordinates of the detected object in the robot's reference frame
    
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
 
    @param   center_point -> Tuple representing the coordinates of the center point
    
    @return  les 4 corners qui définissent la safety zone
    '''
    
    # Distance from the center point to the corners
    distance = 5
    
    # Calculate coordinates of the four corners
    corner1 = (center_point[0] + distance, center_point[1] + distance)
    corner2 = (center_point[0] + distance, center_point[1] - distance)
    corner3 = (center_point[0] - distance, center_point[1] - distance)
    corner4 = (center_point[0] - distance, center_point[1] + distance)
    
    return [corner1, corner2, corner3, corner4]


def local_nav(robot_centroid, robot_direction, prox_sensors):
    object_robot_coords = object_robot_coordinates(prox_sensors)
    object_ground_coords = robot_to_ground_coords(robot_centroid, robot_direction, object_robot_coords)
    Object = safety_zone(object_ground_coords)
    return Object


prox_sensors = [0, 2500, 0, 0, 0]
robot_centroid = [200,-401.8]
robot_direction = (-1,-2)
object_robot_coords = object_robot_coordinates(prox_sensors)
print("object_robot_coords")
print(object_robot_coords)
object_ground_coords = robot_to_ground_coords(robot_centroid, robot_direction, object_robot_coords)
print("object_ground_coords")
print(object_ground_coords)
liste_point = safety_zone(object_ground_coords)

print(liste_point)

#plt.figure(figsize=(8, 6))
fig, ax = plt.subplots()
plt.scatter(liste_point[0][0], liste_point[0][1], color='blue')
plt.scatter(liste_point[1][0], liste_point[1][1], color='blue')
plt.scatter(liste_point[2][0], liste_point[2][1], color='blue')
plt.scatter(liste_point[3][0], liste_point[3][1], color='blue')

plt.scatter(robot_centroid[0], robot_centroid[1], color='green')
ax.quiver(robot_centroid[0], robot_centroid[1], robot_direction[0], robot_direction[1], angles='xy', scale_units='xy', scale=1, color='green', label='Vector')
plt.scatter(object_ground_coords[0], object_ground_coords[1], color='red')
plt.show()
