#skeleton of main code

global_state = 'rotation'
rotation_ended = False
translation_ended = False
local_ended = False

### initialisation
# vision (détection robot, obstacles, goal)
# global nav

iter = 0
while(True):
    iter = iter + 1

    # vision et gobale toutes les n iterations (i modulo n == 0)
    
    # get robot sensor values
    # test if local (with proximity)
    # Kalman (with wheel speed and camera position)


    if global_state == 'rotation':
        # call rotation fct
        if rotation_ended:
            global_state = 'translation'

    elif global_state == 'translation':
        # call translation fct
        if rotation_ended:
            global_state = 'rotation'

    elif global_state == 'local_avoidance':
        # call local_avoidance fct
        if local_ended:
            global_state = 'rotation'

