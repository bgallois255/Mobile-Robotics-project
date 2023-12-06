'''
geometry.py
basic geometry functions
Authors: Benoît Gallois, Jehan Corcelle, Arto Dubuisson, Raphaël Dousson
'''
import math
import matplotlib.pyplot as plt
from matplotlib.legend_handler import HandlerTuple
import numpy as np
import time
from shapely.geometry import Point, Polygon, LineString, MultiPolygon


def center_angle(angle):
    '''
    compute the equivalent angle between -pi and pi 

    Parameter: - angle: angle to be centered in [rad]
    Return: - angle: centered angle in [rad]
    '''
    while angle <= -math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle
'''
def arctan(x, y):
    ''''''
    compute the arctangent from x over y

    Parameter: - x: adjacent in [mm]
               - y: opposite in [mm]
    Return: - angle: angle between -pi and pi in [rad]
    ''''''
    if(y == 0):         # avoid division by 0
        if(x < 0):
            angle = math.pi
        elif(x >= 0):
            angle = 0
    elif(x==0):
        if(y < 0):
            angle = - math.pi / 2
        elif(y > 0):
            angle = math.pi / 2
    else:
        angle = math.atan(x/y)
        if (x < 0 and y > 0):
            angle = math.pi + angle
        elif (x < 0 and y < 0):
            angle = - math.pi + angle

    return angle

print(arctan(0,-1))
'''



def is_inside(point, obstacles):
    for obstacle in range(len(obstacles)):
        min_x = min(s[0] for s in obstacles[obstacle])
        max_x = max(s[0] for s in obstacles[obstacle])
        min_y = min(s[1] for s in obstacles[obstacle])
        max_y = max(s[1] for s in obstacles[obstacle])
        for summit in range(len(obstacles[obstacle])):
            if min_x <= point[0] <= max_x and min_y <= point[1] <= max_y:
                return True
    return False

'''
obstacles = [[(1,2), (1,3), (2, 2), (2,3)],[(4,5), (6,7), (4,7), (6,5)]]
point = (1.1, 2.2)
print(is_inside(point, obstacles))

obstacles = [Polygon([(763, 781), (1035, 781), (1035, 321), (763, 321)])]
goal = [1515.,  542.]
robot_direction = (108, -21)
robot = [345, 576]
start = [345, 576]
shortest_path = [(345.0, 576.0), (763.0, 781.0), (1035.0, 781.0), (1515.0, 542.0)]



plt.figure(figsize=(10, 10))
plt.gca().set_aspect('equal', adjustable='box')
plt.xlim(0, 1920)
plt.ylim(0, 1080)

for i, obstacle in enumerate(obstacles):
    # Extract the exterior coordinates of the obstacle
    obstacle_array = np.array(obstacle.exterior.xy).T
    if i == 0:
        plt.fill(obstacle_array[:, 0], obstacle_array[:, 1], color='gray', alpha=0.7, label='Obstacles')
    else:
        plt.fill(obstacle_array[:, 0], obstacle_array[:, 1], color='gray', alpha=0.7)

plt.plot(start[0], start[1], 'ro', markersize=10, label='Start')
robot_point, = plt.plot(robot[0], robot[1], 'bo', markersize=10, label='Robot')  # Use ',' to unpack the tuple
plt.plot(goal[0], goal[1], 'go', markersize=10, label='Goal')


if shortest_path and shortest_path != "Path not reachable":
    # Plot the shortest path segment by segment
    for i in range(len(shortest_path) - 1):
        node1 = shortest_path[i]
        node2 = shortest_path[i + 1]
        if i == 0:
            plt.plot([node1[0], node2[0]], [node1[1], node2[1]], 'r-', linewidth=2, label='Shortest Path')
        else:
            plt.plot([node1[0], node2[0]], [node1[1], node2[1]], 'r-', linewidth=2)

plt.legend()
plt.draw()
plt.pause(1)

def visualization_robot_position(robot_point, robot):
    
    #@description: this function simply plots the results in the virtual environment
    
    robot_point.set_data(robot[0], robot[1])

    plt.legend()
    plt.draw()
    plt.pause(1)

robot[0] = robot[0] + 10
robot[1] = robot[1] + 10
visualization_robot_position(robot_point, robot)

robot[0] = robot[0] + 10
robot[1] = robot[1] + 10
visualization_robot_position(robot_point, robot)

robot[0] = robot[0] + 10
robot[1] = robot[1] + 10
visualization_robot_position(robot_point, robot)
plt.clf()
plt.pause(2)
plt.show
'''

def plot_initial(obstacles, start, robot, goal, shortest_path):
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlim(0, 1920)
    plt.ylim(0, 1080)

    for i, obstacle in enumerate(obstacles):
        # Extract the exterior coordinates of the obstacle
        obstacle_array = np.array(obstacle.exterior.xy).T
        if i == 0:
            plt.fill(obstacle_array[:, 0], obstacle_array[:, 1], color='gray', alpha=0.7, label='Obstacles')
        else:
            plt.fill(obstacle_array[:, 0], obstacle_array[:, 1], color='gray', alpha=0.7)

    plt.plot(start[0], start[1], 'ro', markersize=10, label='Start')
    robot_point, = plt.plot(robot[0], robot[1], 'bo', markersize=10, label='Robot')  # Use ',' to unpack the tuple
    plt.plot(goal[0], goal[1], 'go', markersize=10, label='Goal')

    if shortest_path and shortest_path != "Path not reachable":
        # Plot the shortest path segment by segment
        for i in range(len(shortest_path) - 1):
            node1 = shortest_path[i]
            node2 = shortest_path[i + 1]
            if i == 0:
                plt.plot([node1[0], node2[0]], [node1[1], node2[1]], 'r-', linewidth=2, label='Shortest Path')
            else:
                plt.plot([node1[0], node2[0]], [node1[1], node2[1]], 'r-', linewidth=2)

    plt.legend()
    plt.draw()
    plt.pause(1)
    return robot_point

def visualization_robot_position(robot_point, robot):
    """
    @description: this function simply plots the results in the virtual environment
    """
    robot_point.set_data(robot[0], robot[1])  # Update the data of the first Line2D object in the list

    plt.legend()
    plt.draw()
    plt.pause(1.5)

# Example usage:

plt.figure(figsize=(10, 10))

obstacles = [Polygon([(763, 781), (1035, 781), (1035, 321), (763, 321)])]
goal = [1515.,  542.]
robot_direction = (108, -21)
robot = [345, 576]
start = [345, 576]
shortest_path = [(345.0, 576.0), (763.0, 781.0), (1035.0, 781.0), (1515.0, 542.0)]

robot_point = plot_initial(obstacles, start, robot, goal, shortest_path)

# Example: Update the robot position

robot[0] = robot[0] + 10
robot[1] = robot[1] + 10
visualization_robot_position(robot_point, robot)
robot[0] = robot[0] + 10
robot[1] = robot[1] + 10
visualization_robot_position(robot_point, robot)
robot[0] = robot[0] + 10
robot[1] = robot[1] + 10
visualization_robot_position(robot_point, robot)

obstacles.append(Polygon([(100, 100), (300, 100), (300, 300), (100, 300)]))
plt.clf()
robot_point = plot_initial(obstacles, start, robot, goal, shortest_path)

robot[0] = robot[0] + 10
robot[1] = robot[1] + 10
visualization_robot_position(robot_point, robot)
robot[0] = robot[0] + 10
robot[1] = robot[1] + 10
visualization_robot_position(robot_point, robot)
robot[0] = robot[0] + 10
robot[1] = robot[1] + 10
visualization_robot_position(robot_point, robot)
