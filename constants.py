'''
constants.py
constants definition
Authors: Benoît Gallois, Jehan Corcelle, Arto Dubuisson, Raphaël Dousson
'''

WHEELS_DIST = 93                        # distance between the wheels of Thymio in [mm]
WHEEL_DIAMETER = 42                     # wheel diameter in [mm]
WHEEL_RADIUS = WHEEL_DIAMETER / 2       # wheel radius in [mm]
ROTATION_CORR = 20                       # correction factor for rotations

MAX_THYMIO_SPEED = 50                   # thymio speed
THYMIO_SPEED_TO_MM_PER_S = 18.1818      # thymio speed to mm/s conversion factor (experimental)

THRESHOLD_PROX_SENSORS = 20             # threshold to detect an obstacle with proximity sensors
