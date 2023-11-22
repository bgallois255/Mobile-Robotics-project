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
    while(angle > math.pi or angle <= - math.pi):
        if(angle > math.pi):
            angle -= 2*math.pi
        elif(angle <= - math.pi):
            angle += 2*math.pi

    return angle

def arctan(x, y):
    '''
    compute the arctangent from x over y

    Parameter: - x: adjacent in [mm]
               - y: opposite in [mm]
    Return: - angle: angle between -pi and pi in [rad]
    '''
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