'''
geometry.py
basic geometry functions
Authors: Benoît Gallois, Jehan Corcelle, Arto Dubuisson, Raphaël Dousson
'''
import math

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

obstacles = [[(1,2), (1,3), (2, 2), (2,3)],[(4,5), (6,7), (4,7), (6,5)]]
point = (1.1, 2.2)
print(is_inside(point, obstacles))