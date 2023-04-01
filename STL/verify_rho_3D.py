import pandas as pd
import numpy as np
from verify_rho import *


def verify_stlcg_rho(PWL):
    A = np.array([[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]])
    x0 = np.array([-0.6, -0.6, 0.1]) # x,y,z, vx, vy, vz, ax,ay, az
    goal = [0.6, 0.6, 0.7]
    T = 20
    b1 = np.array([0.5, -0.1, 0.5, -0.2, -0.1, 0.4], dtype = np.float64)
    B1 = (A, b1)
    b2 = np.array([-0.1, 0.55, 0.5, -0.1, -0.1, 0.5], dtype = np.float64)
    B2 = (A, b2)
    b3 = np.array([-0.1, 0.5, -0.25, 0.5, -0.15, 0.55], dtype = np.float64)
    B3 = (A, b3)
    c = np.array([0.55, -0.05, 0.05, 0.35, -0.2, 0.45], dtype = np.float64)
    C = (A, c)

    In_B1 = in_3Drectangle_formula(A, b1)
    In_B2 = in_3Drectangle_formula(A, b2)
    In_B3 = in_3Drectangle_formula(A, b3)
    In_C = in_3Drectangle_formula(A, c)


    phi_1 = In_B2.eventually(0, T).always(0, 2)
    phi_2 = In_B3.eventually(0, T).always(0, 2)
    phi_3 = In_C.negation().always(0, T)
    phi_4 = In_B1.negation().always(0, T)

    full_specification = phi_4.conjunction(phi_3.conjunction(phi_2.conjunction(phi_1)))

    full_rho = full_specification.robustness(PWL, 0)

    print("full rho = %s \n" %full_rho)
    print("phi1_rho = %s \n" %phi_1.robustness(PWL, 0))
    print("phi2_rho = %s \n" %phi_2.robustness(PWL, 0))
    print("phi3_rho = %s \n" %phi_3.robustness(PWL, 0))
    print("phi4_rho = %s \n" %phi_4.robustness(PWL, 0))


def verify_Ltunnel_rho(PWL):

    T = 10. 
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


    In_B1 = in_3Drectangle_formula(A, b1)
    In_B2 = in_3Drectangle_formula(A, b2)
    In_B3 = in_3Drectangle_formula(A, b3)
    In_B4 = in_3Drectangle_formula(A, b4)
    In_B5 = in_3Drectangle_formula(A, b5)
    In_B6 = in_3Drectangle_formula(A, b6)
    In_B7 = in_3Drectangle_formula(A, b7)
    In_B8 = in_3Drectangle_formula(A, b8)

    phi_1 = In_B1.negation().always(0, T)
    phi_2 = In_B2.negation().always(0, T)
    phi_3 = In_B3.negation().always(0, T)
    phi_4 = In_B4.negation().always(0, T)
    phi_5 = In_B5.negation().always(0, T)
    phi_6 = In_B6.negation().always(0, T)
    phi_7 = In_B7.negation().always(0, T)
    phi_8 = In_B8.negation().always(0, T)

    full_specification = phi_8.conjunction(phi_7.conjunction(phi_6.conjunction(phi_5.conjunction
                        (phi_4.conjunction(phi_3.conjunction(phi_2.conjunction(phi_1)))))))

    full_rho = full_specification.robustness(PWL, 0)

    print("full rho = %s \n" %full_rho)

def verify_Ztunnel_rho(PWL):

    T = 10. 
    A = np.array([[-1, 0, 0],
                [1, 0, 0],
                [0, -1, 0],
                [0, 1, 0],
                [0, 0, -1],
                [0, 0, 1]])

    b1 = np.array([ 0, 50, 0, 50, 1, 0])*0.1
    b2 = np.array([-30, 50, 0, 45, 1, 50])*0.1   
    b3 = np.array([0, 50, -45, 50, 1, 45])*0.1
    b4 = np.array([0, 25, 0, 50, -45, 50])*0.1
    b5 = np.array([0, 30, -5, 45, 0, 45])*0.1
    b6 = np.array([0, 25, 0, 5, -5, 45])*0.1


    In_B1 = in_3Drectangle_formula(A, b1)
    In_B2 = in_3Drectangle_formula(A, b2)
    In_B3 = in_3Drectangle_formula(A, b3)
    In_B4 = in_3Drectangle_formula(A, b4)
    In_B5 = in_3Drectangle_formula(A, b5)
    In_B6 = in_3Drectangle_formula(A, b6)

    phi_1 = In_B1.negation().always(0, T)
    phi_2 = In_B2.negation().always(0, T)
    phi_3 = In_B3.negation().always(0, T)
    phi_4 = In_B4.negation().always(0, T)
    phi_5 = In_B5.negation().always(0, T)
    phi_6 = In_B6.negation().always(0, T)


    full_specification = phi_6.conjunction(phi_5.conjunction
                        (phi_4.conjunction(phi_3.conjunction(phi_2.conjunction(phi_1)))))

    full_rho = full_specification.robustness(PWL, 0)

    print("full rho = %s \n" %full_rho)



if __name__ == '__main__':

    data = pd.read_csv("Ztunnel_chuchu.csv",header=None)
    list = np.array(data)
    PWL = []
    for i in range(1, len(list)):
        t = float(list[i][1])
        x = float(list[i][2])
        y = float(list[i][3])
        z = float(list[i][4])
        data = np.array([np.array([x,y,z]),t])
        PWL.append(data)

    verify_Ztunnel_rho(PWL)
    # verify_stlcg_rho(PWL)1
    # verify_Ltunnel_rho(PWL)