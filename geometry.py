'''
geometry.py
basic geometry functions
Authors: Benoît Gallois, Jehan Corcelle, Arto Dubuisson, Raphaël Dousson
'''
import math

# not needed yet, but maybe later if need of acos or asin!!

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

