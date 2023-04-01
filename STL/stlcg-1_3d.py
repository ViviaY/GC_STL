import numpy as np
import matplotlib.pyplot as plt

import gurobipy as gp
from gurobipy import GRB

# from STL import plan, Node
from GurobiSolver import GurobiPlan
from formula import Node
from vis3D import vis as vis
from vis3D_drone import vis as drone_vis

from verify_rho import *
import pandas as pd
import time
def test():
    # x0 = [-1., -1, -1]
    x0 = np.array([-0.6, -0.6, 0.1, 0, 0, 0, 0, 0, 0]) # x,y,z, vx, vy, vz, ax,ay, az
    goal = [0.6, 0.6, 0.7, 0, 0, 0, 0, 0, 0]
    T = 20. 
    vmax = 2
    amax = 0.2
    num_segs = 40
    rho_min=0.02


    wall_half_width = 0.02
    A = np.array([[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]])
    walls = []

    walls.append(np.array([-0.7, -0.7, -0.7, 0.7, 0, 1], dtype = np.float64))
    walls.append(np.array([0.7, 0.7, -0.7, 0.7, 0, 1], dtype = np.float64))
    walls.append(np.array([-0.7, 0.7, -0.7, -0.7, 0, 1], dtype = np.float64))
    walls.append(np.array([-0.7, 0.7, 0.7, 0.7, 0, 1], dtype = np.float64))

    obs = []
    for wall in walls:
        if wall[0]==wall[1]:
            wall[0] -= wall_half_width
            wall[1] += wall_half_width
        elif wall[2]==wall[3]:
            wall[2] -= wall_half_width
            wall[3] += wall_half_width
        else:
            raise ValueError('wrong shape for axis-aligned wall')
        wall *= np.array([-1,1,-1,1,-1,1])
        obs.append((A, wall))


    b1 = np.array([0.5, -0.1, 0.5, -0.2, -0.1, 0.4], dtype = np.float64)
    B1 = (A, b1)
    b2 = np.array([-0.1, 0.55, 0.5, -0.1, -0.1, 0.5], dtype = np.float64)
    B2 = (A, b2)
    b3 = np.array([-0.1, 0.5, -0.25, 0.5, -0.15, 0.55], dtype = np.float64)
    B3 = (A, b3)
    c = np.array([0.55, -0.05, 0.05, 0.35, -0.2, 0.45], dtype = np.float64)
    C = (A, c)

    plots = [[[B1,], 'y'], [[B2,], 'r'], [[B3,], 'g'], [[C,], 'b'], [obs, 'k']]

    notC = Node('negmu', info={'A':C[0], 'b':C[1]})
    notB1 = Node('negmu', info={'A':B1[0], 'b':B1[1]})
    B2 = Node('mu', info={'A':B2[0], 'b':B2[1]})
    B3 = Node('mu', info={'A':B3[0], 'b':B3[1]})

    # phi_11 = Node('F', deps=[Node('A', deps=[B1,], info={'int':[0,2]}),], info={'int':[0,T]})
    phi_1 = Node('F', deps=[Node('A', deps=[B2,], info={'int':[0,2]}),], info={'int':[0,T]})
    phi_2 = Node('F', deps=[Node('A', deps=[B3,], info={'int':[0,2]}),], info={'int':[0,T]})
    phi_3 = Node('A', deps=[notC], info={'int':[0,T]})
    phi_4 = Node('A', deps=[notB1], info={'int':[0,T]})

    spec = Node('and', deps=[phi_1, phi_2, phi_3, phi_4]) 

    x0s = [x0, ]
    specs = [spec, ]
    goals = [goal, ]

    # space constraints
    bloat = 0.01
    min_lim= np.array([-0.7+bloat, -0.7+bloat, 0+bloat, -vmax, -vmax, -vmax, -amax,-amax,-amax],dtype =np.float64)
    max_lim= np.array([0.7-bloat, 0.7-bloat, 1-bloat, vmax, vmax, vmax, amax,amax,amax],dtype =np.float64)
    limits = [min_lim, max_lim]

    # PWL = plan(x0s, specs, limits, goals, limits=limits, num_segs=9, tmax=tmax, vmax=vmax)
    solver = GurobiPlan(x0s, specs, limits, goals, num_segs=num_segs, tmax=T, vmax=vmax, amax=amax, rho_min=rho_min)
    time_start=time.time()
    PWL, rho = solver.Solve()
    time_end=time.time()
    print('time cost',time_end-time_start,'s')
    print("rho = ", rho)
    # data = pd.read_csv("stlcg-1_3d_chuchu.csv",header=None)
    # list = np.array(data)
    # PWL = []
    # for i in range(1, len(list)):
    #     t = float(list[i][2])
    #     x = float(list[i][3])
    #     y = float(list[i][4])
    #     z = float(list[i][5])
    #     data = np.array([np.array([x,y,z]),t])
    #     PWL.append(data)
     # Evaluating the Trajectories
    if PWL != None:
        In_B1 = in_3Drectangle_formula(A, b1)
        In_B2 = in_3Drectangle_formula(A, b2)
        In_B3 = in_3Drectangle_formula(A, b3)
        In_C = in_3Drectangle_formula(A, c)


        phi_1 = In_B2.eventually(0, T).always(0, 2)
        phi_2 = In_B3.eventually(0, T).always(0, 2)
        phi_3 = In_C.negation().always(0, T)
        phi_4 = In_B1.negation().always(0, T)


        full_specification = phi_4.conjunction(phi_3.conjunction(phi_2.conjunction(phi_1)))#
        # full_specification = phi_2
        full_rho = full_specification.robustness(PWL, 0)

        print("full rho = %s \n" %full_rho)
        print("phi1_rho = %s \n" %phi_1.robustness(PWL, 0))
        print("phi2_rho = %s \n" %phi_2.robustness(PWL, 0))
        print("phi3_rho = %s \n" %phi_3.robustness(PWL, 0))
        print("phi4_rho = %s \n" %phi_4.robustness(PWL, 0))


    # # write the output trajectory and time to the csv file
    if PWL[0] is not None: 
        dataframe=pd.DataFrame(columns=['full_rho', 't', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az']) 
        for i in range(0, len(PWL)-1):
            data = np.array([rho, PWL[i][1], \
                            PWL[i][0][0], PWL[i][0][1],PWL[i][0][2], \
                            PWL[i][0][3], PWL[i][0][4],PWL[i][0][5], \
                            PWL[i][0][6], PWL[i][0][7],PWL[i][0][8]], dtype=np.float64)
            dataframe.loc[i] = data
        

        dataframe.to_csv("stlcg-1_3d.csv",index=True,sep=',')

    return x0s, plots, PWL




if __name__ == '__main__':
    # x0s, plots, PWL = test()
    results = vis(test)
    # results = drone_vis(test)

