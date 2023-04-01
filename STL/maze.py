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

def test():
    x0 = np.array([1., 0.5, 0.5, 0, 0, 0, 0, 0, 0]) # x,y,z, vx, vy, vz, ax,ay, az
    goal = np.array([39, 2., 2., 0, 0, 0, 0, 0, 0])*0.1

    T = 20. 
    vmax = 10.
    amax = 10.
    num_segs = 40
    rho_min=0.2


    wall_half_width = 0.02
    A = np.array([[-1, 0, 0],
                    [1, 0, 0],
                    [0, -1, 0],
                    [0, 1, 0],
                    [0, 0, -1],
                    [0, 0, 1]])

    b = []
    b.append(np.array([0, 45, 5, 10, 10, -5])*0.1)
    b.append(np.array([5, 0,5, 10, 10, 30])*0.1)
    b.append(np.array([-45, 50, 5, 10, 10, 30])*0.1)
    b.append(np.array([0, 25, 5, 10, 5, 0])*0.1)
    b.append(np.array([-30, 35, 5, 10, 0, 10])*0.1)
    b.append(np.array([-25, 30, 0, 10, 0, 10])*0.1)
    b.append(np.array([-35, 45, 5, 0, 0, 10])*0.1)
    b.append(np.array([-30,45,5,5,-10,25])*0.1)
    b.append(np.array([-0, 45, -5, 10, -20, 25])*0.1)
    b.append(np.array([-0, 45, 5, 0, -20, 25])*0.1)
    b.append(np.array([-15, 25, 5, 10, 0, 20])*0.1)
    b.append(np.array([0, 10, 0, 5 ,-5, 25])*0.1)
    b.append(np.array([0, 15, 5, 0, 0, 20])*0.1)
    b.append(np.array([0, 15, -5, 10, 0, 20])*0.1)
    b.append(np.array([0, 45, 5, 10, -25, 30])*0.1)
    b.append(np.array([5, 50, 10, -5, 10, 30])*0.1)
    b.append(np.array([5, 50, -10, 15, 10, 30])*0.1)
    
    B = []
    for i in range (17):
        B.append((A,b[i]))
    # B1 = (A_tri2, b1)
    # B2 = (A_tri2, b2)
    # B3 = (A_tri1, b3)
    # B4 = (A_tri1, b4)
    # B5 = (A_tri1, b5)
    # B6 = (A, b6)
    # B7 = (A, b7)
    # B8 = (A, b8)
    # B9 = (A, b9)

    pb = []
    pb.append(np.array([0, 15, 0, 5, 0, 5])*0.1)
    pb.append(np.array([-10, 15, 0, 5, -5, 20])*0.1)
    pb.append(np.array([-10, 30, 0, 5, -20, 25])*0.1)
    pb.append(np.array([-25, 30, 5, 10, -10, 20])*0.1)
    pb.append(np.array([-25, 30, 5, 0, 5, 10])*0.1)
    pb.append(np.array([-30, 45, -5, 10, -10, 20])*0.1)
    pb.append(np.array([-30, 45, 5, 10, 5, 0])*0.1)
    pb.append(np.array([-35, 45, 0, 10, 0, 10])*0.1)
    PB = []
    for i in range (8):
        PB.append((A,pb[i]))

    plots = []
    for i in range(8):
        plots.append([[PB[i],], 'y'])

    BNode = []
    for i in range(17):
        BNode.append(Node('negmu', info={'A':B[i][0], 'b':B[i][1]}))
    # B1 = Node('negmu', info={'A':B1[0], 'b':B1[1]})
    # B2 = Node('negmu', info={'A':B2[0], 'b':B2[1]})
    # B3 = Node('negmu', info={'A':B3[0], 'b':B3[1]})
    # B4 = Node('negmu', info={'A':B4[0], 'b':B4[1]})
    # B5 = Node('negmu', info={'A':B5[0], 'b':B5[1]})
    # B6 = Node('negmu', info={'A':B6[0], 'b':B6[1]})
    # B7 = Node('negmu', info={'A':B7[0], 'b':B7[1]})
    # B8 = Node('negmu', info={'A':B8[0], 'b':B8[1]})
    # B9 = Node('negmu', info={'A':B9[0], 'b':B9[1]})
  

    # # phi_1 = Node('F', deps=[Node('A', deps=[B1,], info={'int':[0,2]}),], info={'int':[0,T]})
    # phi_1 = Node('A', deps=[B1,], info={'int':[0,T]})
    # phi_2 = Node('A', deps=[B2,], info={'int':[0,T]})
    # phi_3 = Node('A', deps=[B3,], info={'int':[0,T]})
    # phi_4 = Node('A', deps=[B4,], info={'int':[0,T]})
    # phi_5 = Node('A', deps=[B5,], info={'int':[0,T]})
    # phi_6 = Node('A', deps=[B6,], info={'int':[0,T]})
    # phi_7 = Node('A', deps=[B4,], info={'int':[0,T]})
    # phi_8 = Node('A', deps=[B5,], info={'int':[0,T]})
    # phi_9 = Node('A', deps=[B6,], info={'int':[0,T]})
    phi = []
    for i in range(17):
        phi.append(Node('A', deps=[BNode[i],], info={'int':[0,T]}))

    spec = Node('and', deps=[phi[i] for i in range(3)]) 
    # spec = Node('and', deps=[phi_2,]) 


    x0s = [x0, ]
    specs = [spec, ]
    goals = [goal, ]

    # space constraints
    bloat = wall_half_width+0.01
    # min_lim= np.array([-0.7+bloat, -0.7+bloat, 0+bloat, -vmax, -vmax, -vmax, -amax,-amax,-amax],dtype =np.float64)
    min_lim = np.array([-3+bloat, 0+bloat, 0+bloat, -vmax, -vmax, -vmax, -amax,-amax,-amax],dtype =np.float64)
    max_lim= np.array([5.1-bloat, 3-bloat, 5.1-bloat, vmax, vmax, vmax, amax,amax,amax],dtype =np.float64)
    limits = [min_lim, max_lim]

    # PWL = plan(x0s, specs, limits, goals, limits=limits, num_segs=9, tmax=tmax, vmax=vmax)
    solver = GurobiPlan(x0s, specs, limits, goals, num_segs=num_segs, tmax=T, vmax=vmax, amax=amax, rho_min=rho_min)
    
    PWL, rho = solver.Solve()
    print("rho = ", rho)

    #  # Evaluating the Trajectories
    # if PWL != None:
    #     In_B1 = in_3Drectangle_formula(A, b1)
    #     In_B2 = in_3Drectangle_formula(A, b2)
    #     In_B3 = in_3Drectangle_formula(A, b3)
    #     In_C = in_3Drectangle_formula(A, b4)


    #     phi_1 = In_B2.eventually(0, T).always(0, 2)
    #     phi_2 = In_B3.eventually(0, T).always(0, 2)
    #     phi_3 = In_C.negation().always(0, T)
    #     phi_4 = In_B1.negation().always(0, T)


    #     full_specification = phi_4.conjunction(phi_3.conjunction(phi_2.conjunction(phi_1)))
    #     # full_specification = phi_2
    #     full_rho = full_specification.robustness(PWL, 0)

    #     print("full rho = %s \n" %full_rho)
    #     print("phi1_rho = %s \n" %phi_1.robustness(PWL, 0))
    #     print("phi2_rho = %s \n" %phi_2.robustness(PWL, 0))
    #     print("phi3_rho = %s \n" %phi_3.robustness(PWL, 0))
    #     print("phi4_rho = %s \n" %phi_4.robustness(PWL, 0))


    # write the output trajectory and time to the csv file
    if PWL[0] is not None: 
        dataframe=pd.DataFrame(columns=['full_rho', 't', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az']) 
        for i in range(0, len(PWL)-1):
            data = np.array([rho, PWL[i][1], \
                            PWL[i][0][0], PWL[i][0][1],PWL[i][0][2], \
                            PWL[i][0][3], PWL[i][0][4],PWL[i][0][5], \
                            PWL[i][0][6], PWL[i][0][7],PWL[i][0][8]], dtype=np.float64)
            dataframe.loc[i] = data
        

        dataframe.to_csv("zigzig_3d.csv",index=True,sep=',')

    return x0s, plots, PWL




if __name__ == '__main__':
    # x0s, plots, PWL = test()
    results = vis(test)
    # results = drone_vis(test)

    # fig = plt.figure()
    # axes = fig.add_subplot(111, projection='3d')
    # axes = a3.Axes3D(plt.figure())
    # A = np.array([[-1, 0, 0],
    #                 [1, 0, 0],
    #                 [0, -1, 0],
    #                 [0, 1, 0],
    #                 [0, 0, -1],
    #                 [0, 0, 1]])

