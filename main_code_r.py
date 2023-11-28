#skeleton of main code test
'''
main_code_r.py
skeleton of main code
Authors: Benoît Gallois, Jehan Corcelle, Arto Dubuisson, Raphaël Dousson
'''

# import libraries
import time
from tdmclient import ClientAsync

# import project files
import constants as cst
import extended_Kalman_filter as eKf
import vision

def main_function():
    # state variables
    global_state = 'rotation'
    rotation_ended = False
    translation_ended = False

    ### initialisation
    # vision (détection robot, obstacles, goal)
    obstacles, goal_centroid, robot_centroid, robot_direction = vision.vision_obstacles_positions()
    # global nav
    


    old_time = 0

    iter = 0
    while(True):
        # vision et gobale toutes les n iterations (i modulo n == 0)
        
        # get robot sensor values
        # test if local (with proximity)

        if iter == 0:                   # first time
            old_time = time.time()
        dt = time.time() - old_time     # get time difference for Kalman
        # Kalman (with wheel speed and camera position)
        old_time = time.time()

        if global_state == 'rotation':
            if rotation_ended:
                global_state = 'translation'
            # call rotation fct

        elif global_state == 'translation':
            # if goal reached
                # exit
            if translation_ended:
                global_state = 'rotation'
            
            # if rotation not correct
                # global_state = 'rotation'

            # call translation fct

        elif global_state == 'local_avoidance':
            # call local_avoidance fct (= add obstacle on global map and recompute global nav)
            
            global_state = 'rotation'

        iter = iter + 1

        