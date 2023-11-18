import numpy as np
import math

import extended_Kalman_filter as eKf

Kal = eKf.Kalman(0,0,0)
print(Kal.B)
Kal.B = 1
print(Kal.B)
A,G = Kal.matrices_update(0.1, 50, 50)
print(A)
print(G)
print(A[1][0])
print(Kal.Mu[2][0])