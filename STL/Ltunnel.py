import numpy as np
import matplotlib.pyplot as plt

import gurobipy as gp
from gurobipy import GRB

# from STL import plan, Node
from GurobiSolver import GurobiPlan
from formula import Node
from vis3D import vis as vis
from vis3D_drone import vis as drone_vis
from drawRobustnessTube import  drawtube
from verify_rho_3D import verify_Ltunnel_rho
import pandas as pd
import time
def test():
    
    x0 = np.array([2, 2, 2, 0, 0, 0, 0, 0, 0])*0.1
    goal = np.array([39, 2, 2.5, 0, 0, 0, 0, 0, 0])*0.1
    T = 20. 
    vmax = 2
    amax = 0.2
    num_segs = 40
    rho_min = 0.2

    wall_half_width = 0.02
    A = np.array([[-1, 0, 0],
                [1, 0, 0],
                [0, -1, 0],
                [0, 1, 0],
                [0, 0, -1],
                [0, 0, 1]])

    b1 = np.array([0, 40, 0, 5, -25, 40])*0.1
    b2 = np.array([0, 10, 0, 5, -5, 25])*0.1
    b3 = np.array([-30, 40, 0, 5, -5, 25])*0.1
    b4 = np.array([-15, 25, 0, 5, 0, 20])*0.1
    b5 = np.array([0, 40, 0, 5, 5, 0])*0.1
    b6 = np.array([0, 40, 0, 5, -40, 45])*0.1
    b7 = np.array([0, 40, -5, 10, 0, 40])*0.1
    b8 = np.array([0, 40, 5, 0, 0, 40])*0.1

    B1 = (A, b1)
    B2 = (A, b2)
    B3 = (A, b3)
    B4 = (A, b4)
    B5 = (A, b5)
    B6 = (A, b6)
    B5 = (A, b7)
    B6 = (A, b8)

    # obs = [(A, b1),
    #         (A, b2),
    #         (A, b3),
    #         (A, b4),
    #         (A, b5),
    #         (A, b6)]

    pb1 = np.array([[0], [15], [0], [5], [0], [5]])*0.1
    pb2 = np.array([[-10], [15], [0], [5], [-5], [20]])*0.1
    pb3 = np.array([[-10], [30], [0], [5], [-20], [25]])*0.1
    pb4 = np.array([[-25], [30], [0], [5], [-5], [20]])*0.1
    pb5 = np.array([[-25], [40], [0], [5], [0], [5]])*0.1
    pB1 = (A, pb1)
    pB2 = (A, pb2)
    pB3 = (A, pb3)
    pB4 = (A, pb4)
    pB5 = (A, pb5)
    # plots = [[[pB1,], 'y'], [[pB2,], 'y'], [[pB3,], 'y'], [[pB4,], 'y']]

    # plots = [[[B1,], 'r'], [[B2,], 'r'], [[B3,], 'r'], [[B4,], 'r'], [[B5,], 'r'], [[B6,], 'r'],
    plots = [[[pB1,], 'y'], [[pB2,], 'y'], [[pB3,], 'y'], [[pB4,], 'y'],[[pB5,], 'y']]

    B1 = Node('negmu', info={'A':B1[0], 'b':B1[1]})
    B2 = Node('negmu', info={'A':B2[0], 'b':B2[1]})
    B3 = Node('negmu', info={'A':B3[0], 'b':B3[1]})
    B4 = Node('negmu', info={'A':B4[0], 'b':B4[1]})
    B5 = Node('negmu', info={'A':B5[0], 'b':B5[1]})
    B6 = Node('negmu', info={'A':B6[0], 'b':B6[1]})
  
    phi_1 = Node('A', deps=[B1,], info={'int':[0,T]})
    phi_2 = Node('A', deps=[B2,], info={'int':[0,T]})
    phi_3 = Node('A', deps=[B3,], info={'int':[0,T]})
    phi_4 = Node('A', deps=[B4,], info={'int':[0,T]})
    phi_5 = Node('A', deps=[B5,], info={'int':[0,T]})
    phi_6 = Node('A', deps=[B6,], info={'int':[0,T]})


    spec = Node('and', deps=[phi_1, phi_2, phi_3, phi_4, phi_5, phi_6]) 
    # spec = Node('and', deps=[phi_2]) 


    x0s = [x0, ]
    specs = [spec, ]
    goals = [goal, ]

    # space constraints
    bloat = wall_half_width+0.05
    min_lim= np.array([0+bloat,0+bloat, 0+bloat, -vmax, -vmax, -vmax, -amax,-amax,-amax],dtype =np.float64)
    max_lim= np.array([50*0.1-bloat, 50*0.1-bloat, 50*0.1-bloat, vmax, vmax, vmax, amax,amax,amax],dtype =np.float64)
    limits = [min_lim, max_lim]

    solver = GurobiPlan(x0s, specs, limits, goals, num_segs=num_segs, tmax=T, vmax=vmax, amax=amax, rho_min=rho_min)
    # time_start=time.time()
    PWL, rho = solver.Solve()
    time_end=time.time()
    # print('time cost',time_end-time_start,'s')
    print("rho = ", rho)

    #  # Evaluating the Trajectories
    if PWL != None:
        verify_Ltunnel_rho(PWL)

        # In_B1 = in_3Drectangle_formula(A, b1)
        # In_B2 = in_3Drectangle_formula(A, b2)
        # In_B3 = in_3Drectangle_formula(A, b3)
        # In_B4 = in_3Drectangle_formula(A, b4)
        # In_B5 = in_3Drectangle_formula(A, b5)
        # In_B6 = in_3Drectangle_formula(A, b6)

        # phi_1 = In_B1.negation().always(0, T)
        # phi_2 = In_B2.negation().always(0, T)
        # phi_3 = In_B3.negation().always(0, T)
        # phi_4 = In_B4.negation().always(0, T)
        # phi_5 = In_B5.negation().always(0, T)
        # phi_6 = In_B6.negation().always(0, T)

        # full_specification = phi_6.conjunction(phi_5.conjunction(phi_4.conjunction(phi_3.conjunction(phi_2.conjunction(phi_1)))))
        # full_rho = full_specification.robustness(PWL, 0)

        # print("full rho = %s \n" %full_rho)
        # print("phi1_rho = %s \n" %phi_1.robustness(PWL, 0))
        # print("phi2_rho = %s \n" %phi_2.robustness(PWL, 0))
        # print("phi3_rho = %s \n" %phi_3.robustness(PWL, 0))
        # print("phi4_rho = %s \n" %phi_4.robustness(PWL, 0))
        # print("phi5_rho = %s \n" %phi_5.robustness(PWL, 0))
        # print("phi6_rho = %s \n" %phi_6.robustness(PWL, 0))


    # write the output trajectory and time to the csv file
    # if PWL[0] is not None: 
    #     dataframe=pd.DataFrame(columns=['full_rho', 't', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az']) 
    #     for i in range(0, len(PWL)-1):
    #         data = np.array([rho, PWL[i][1], \
    #                         PWL[i][0][0], PWL[i][0][1],PWL[i][0][2], \
    #                         PWL[i][0][3], PWL[i][0][4],PWL[i][0][5], \
    #                         PWL[i][0][6], PWL[i][0][7],PWL[i][0][8]], dtype=np.float64)
    #         dataframe.loc[i] = data
        

    #     dataframe.to_csv("Ltunnel.csv",index=True,sep=',')

    return x0s, plots, PWL




if __name__ == '__main__':
    # x0s, plots, PWL = test()
    results = vis(test)
    # results = drone_vis(test)
