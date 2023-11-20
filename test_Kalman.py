import numpy as np
import math
import matplotlib.pyplot as plt

import extended_Kalman_filter as eKf


dt = 1

state_x = 0
state_y = 0
state_theta = math.pi / 4.0
#state_theta = math.pi / 2.0
state_vr = 0
state_vl = 0

Kfilter = eKf.Kalman(state_x,state_y,state_theta)         #Attention, initialiser Kfilter avec x,y,theta donné par la caméra
                                                          # sinon le filtre met bcp de temps à s'adapter à sa position réelle
max_iter = 12
x1 = np.zeros(max_iter)
y1 = np.zeros(max_iter)
x2 = np.zeros(max_iter)
y2 = np.zeros(max_iter)

for iteration in range(max_iter):
    print(iteration)
    print("Mu")  
    print(Kfilter.Mu)

    x1[iteration] = state_x
    y1[iteration] = state_y

    Kfilter.dt_update(dt)
    Kfilter.measurement_wheels(state_vr, state_vl)
    if(iteration % 3 == 0):
        Kfilter.measurement_position(state_x, state_y, state_theta)
        state_vr = math.sqrt(2)
        state_vl = math.sqrt(2)
        # state_vr = 1
        # state_vl = 1
        print("camera")

    Kfilter.Kalman_filter()

    x2[iteration] = Kfilter.Mu[0][0]
    y2[iteration] = Kfilter.Mu[1][0]

    state_x += 1
    state_y += 1

    
# Plotting the points
plt.scatter(x1, y1, label=' ', color='blue')
plt.scatter(x2, y2, label='Set 2', color='red')

# Adding labels and title
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Scatter Plot of 2D Points')

# Adding a legend
plt.legend()

# Display the plot
plt.show()



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