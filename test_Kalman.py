import numpy as np
import math

import extended_Kalman_filter as eKf

Kfilter = eKf.Kalman(0,0,0)
dt = 1

state_x = 0
state_y = 0
state_theta = math.pi / 4.0
state_vr = 0
state_vl = 0

#state_theta = math.pi / 4


for iteration in range(10):
    print(iteration)

    Kfilter.dt_update(dt)
    Kfilter.measurement_wheels(state_vr, state_vl)
    if(iteration % 2 == 0):
        Kfilter.measurement_position(state_x, state_y, state_theta)
        state_vr = math.sqrt(2)
        state_vl = math.sqrt(2)
        print("camera")
    Kfilter.Kalman_filter()

    state_x += 1
    state_y += 1
    
    print(Kfilter.Mu)




'''
print(Kal.B)
Kal.B = 1
print(Kal.B)
A,G = Kal.model_update()
print(A)
print(G)
print(A[1][0])
print(Kal.Mu[2][0])
'''