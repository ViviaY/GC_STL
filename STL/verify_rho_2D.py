import pandas as pd
import numpy as np
from verify_rho import *

data = pd.read_csv("stlcg-1_2d.csv",header=None)
list = np.array(data)
# list = data.values.tolist()
PWL = []


for i in range(1, len(list)):
    t = float(list[i][1])
    x = float(list[i][2])
    y = float(list[i][3])

    data = np.array([np.array([x,y]),t])
    PWL.append(data)


goal_bounds = (-7,8,-8,9)     # (xmin, xmax, ymin, ymax)
obstacle_bounds = (-3,5,-4,6)
T = 20

A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
In_B = in_2Drectangle_formula(A, goal_bounds)
In_C = in_2Drectangle_formula(A, obstacle_bounds)

phi_1 = In_B.eventually(0, T)
phi_2 = In_C.negation().always(0, T)

full_specification = phi_2.conjunction(phi_1)
full_rho = full_specification.robustness(PWL, 0)

print("full rho = %s \n" %full_rho)
print("phi1_rho = %s \n" %phi_1.robustness(PWL, 0))
print("phi2_rho = %s \n" %phi_2.robustness(PWL, 0))