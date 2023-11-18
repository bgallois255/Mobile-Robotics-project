import numpy as np

import extended_Kalman_filter as eKf

Kal = eKf.Kalman(0,0,0)
print(Kal.B)
Kal.B = 1
print(Kal.B)
