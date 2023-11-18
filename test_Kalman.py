import numpy as np

import extended_Kalman_filter as eKf

Kal = eKf.Kalman()
print(Kal.B)
Kal.B = 1
print(Kal.B)
