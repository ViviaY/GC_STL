import pandas as pd
import numpy as np
from numpy import genfromtxt
from decimal import Decimal
from vis3D import visTwoTraj,visOneTraj
from vis3D_drone import vis as drone_vis
from drawRobustnessTube import  drawtube
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# Interpolations and trajecotry errors analysis
# read the reference traj
def ReadTrajs(ref_file, track_file):
    # data = genfromtxt('stlcg-1_3d.csv', delimiter=',')
    data = genfromtxt(ref_file, delimiter=',')

    ref_list = np.array(data)
    ref_traj = []
    ref_dt  = float(ref_list[2][2])  - float(ref_list[1][2])
    for i in range(1, len(ref_list)):
        t = ref_list[i][2]
        p = np.array(ref_list[i][3:6])
        v =  np.array(ref_list[i][6:9])
        a = np.array(ref_list[i][9:12])
        ref_traj.append([t, p, v, a])

    # read the tracking trajectory
    # data = genfromtxt('real_trajectory.csv', delimiter=',')
    data = genfromtxt(track_file, delimiter=',')

    track_list = np.array(data)
    track_traj = []
    track_dt  = float(track_list[2][0])  - float(track_list[1][0])
    for i in range(len(track_list)):
        t = track_list[i][0]
        p = np.array(track_list[i][1:4])
        track_traj.append([t,p])
    
    return ref_traj, ref_dt, track_traj, track_dt

# Interpolation of the trajectory
def Interpolation(ref_traj, track_dt):
    Interp_traj = []
    inter_t = 0.0
    for i in range(len(ref_traj)-1):
        t1 = ref_traj[i+1][0]
        x0 = ref_traj[i][1]
        v0 = ref_traj[i][2]
        a0 = ref_traj[i][3]
        k = 0
        while True:
            if inter_t < t1:
                inter_p = x0 + (track_dt*k)*v0 + a0*((track_dt*k)**2)/2
                Interp_traj.append([inter_t, inter_p])
                inter_t += track_dt
                inter_t = round(inter_t, 3)
                k += 1
            else:
                break

    Interp_traj.append(ref_traj[-1])

    # # # write the output trajectory and time to the csv file
    # if Interp_traj is not None: 
    #     dataframe=pd.DataFrame(columns=['t', 'x', 'y', 'z']) 
    #     for i in range(0, len(Interp_traj)-1):
    #         data = np.array([Interp_traj[i][0], \
    #                         Interp_traj[i][1][0], Interp_traj[i][1][1],Interp_traj[i][1][2]], dtype=np.float64)
    #         dataframe.loc[i] = data
        

    #     dataframe.to_csv("ztunnel_interp.csv",index=True,sep=',')

    return Interp_traj

# trajectory error
def TrajError(Interp_traj, track_traj):


    n = np.size(track_traj, 0)
    traj_error = np.zeros([n, 2])

    err = 0.0

    for i in range(len(track_traj)):
        err = max(err, np.linalg.norm(Interp_traj[i][1] - track_traj[i][1]))
        traj_error[i,:] = np.array([Interp_traj[i][0], np.linalg.norm(Interp_traj[i][1] - track_traj[i][1])])

    print("*************** err = ", err)
    return traj_error

def env_stcg_1_3d():
     ## visualization
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

    return plots


def env_ztunnel():
    A = np.array([[-1, 0, 0],
                [1, 0, 0],
                [0, -1, 0],
                [0, 1, 0],
                [0, 0, -1],
                [0, 0, 1]])

    pb1 = np.array([[0], [25], [0], [5], [0], [5]]) *0.1
    pb2 = np.array([[-25], [30], [0], [5], [0], [45]])*0.1
    pb3 = np.array([[-25], [30], [0], [50], [-45], [50]])*0.1
    pb4 = np.array([[-30], [50], [-45], [50], [-45], [50]])*0.1
    pB1 = (A, pb1)
    pB2 = (A, pb2)
    pB3 = (A, pb3)
    pB4 = (A, pb4)

    plots = [[[pB1,], 'y'], [[pB2,], 'y'], [[pB3,], 'y'], [[pB4,], 'y']]

    return plots

def env_tunnel():
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

    return plots

def env_zigzag():
    A = np.array([[-1,0,0],
                [1,0,0],
                [0,-1,0],
                [0,1,0],
                [0,0,-1],
                [0,0,1]])
    A_tri1 = np.array([[-1,-1,0],
                        [1,-1,0],
                        [0,1,0],
                        [0,0,-1],
                        [0,0,1]])
    A_tri2 = np.array([[-1,1,0],
                        [1,1,0],
                        [0,-1,0],
                        [0,0,-1],
                        [0,0,1]])


    b1 = np.array([5, 20, 0, 0, 50])*0.1
    b2 = np.array([-10,35,0, 0, 50])*0.1
    b3 = np.array([-30,0,30, 0, 50])*0.1
    b4 = np.array([-15,-15,30,0,50])*0.1
    b5 = np.array([-45, 15,30,0,50])*0.1
    b6 = np.array([15, 50, 1, 0, 0, 50])*0.1
    b7 = np.array([15, 50, -30, 31, 0, 50])*0.1
    b8 = np.array([16, -15, 0, 30, 0, 50])*0.1
    b9 = np.array([-50, 50, 0, 30, 0, 50])*0.1

    # obstacles = [(A_tri2, b1),
    #             (A_tri2, b2),
    #             (A_tri1, b3),
    #             (A_tri1, b4),
    #             (A_tri1, b5),
    #             (A, b6),
    #             (A, b7),
    #             (A, b8),
    #             (A, b9)]

    # b1 = np.array([0.5, -0.1, 0.5, -0.2, -0.1, 0.4], dtype = np.float64) 
    B1 = (A_tri2, b1)
    B2 = (A_tri2, b2)
    B3 = (A_tri1, b3)
    B4 = (A_tri1, b4)
    B5 = (A_tri1, b5)
    B6 = (A, b6)
    B7 = (A, b7)
    B8 = (A, b8)
    B9 = (A, b9)

    plots = [[[B1,], 'y', 'tri'], [[B2,], 'y','tri'], [[B3,], 'y','tri'], [[B4,], 'y','tri'],[[B5], 'y','tri'], 
    [[B6,], 'y'], [[B7,], 'y'], [[B8,], 'y'],[[B9,], 'y']]

    return plots


def test(ref_file,track_file):
    ref_traj, ref_dt, track_traj, track_dt = ReadTrajs(ref_file, track_file)
    # Interp_traj = Interpolation(ref_traj, track_dt)
    # traj_error = TrajError(Interp_traj, track_traj)

    # plt.plot(traj_error[:,0], traj_error[:,1], alpha=0.8, color='g', label='tracking error')
    # plt.plot(traj_error[:,0], 0.2*np.ones(np.size(traj_error,0)), alpha=0.8, color='r', label='robustness')
    # plt.xlabel('t')
    # plt.ylabel('tracking error')
    # plt.legend()

    # plt.show()

    return ref_traj, track_traj


def drawmultipTraj(ref_traj):
    
    # data = genfromtxt('stlcg-1_3d.csv', delimiter=',')
    data = genfromtxt(ref_file, delimiter=',')

    ref_list = np.array(data)
    ref_traj = []
    ref_dt  = float(ref_list[2][2])  - float(ref_list[1][2])
    for i in range(1, len(ref_list)):
        t = ref_list[i][2]
        p = np.array(ref_list[i][3:6])
        v =  np.array(ref_list[i][6:9])
        a = np.array(ref_list[i][9:12])
        ref_traj.append([t, p, v, a])


    file_names = []
    for i in range(1, 5):
        string = "ztunnel_rand_initial/ztunnel_gc_real_traj_rand"+str(i)+".csv"
        file_names.append(string)


    trajs = []
    for file in file_names:
        data = genfromtxt(file, delimiter=',')

        track_list = np.array(data)
        track_traj = []
        track_dt  = float(track_list[2][0])  - float(track_list[1][0])
        for i in range(len(track_list)):
            t = track_list[i][0]
            p = np.array(track_list[i][1:4])
            track_traj.append([t,p])
        
        trajs.append(track_traj)


    # plots = env_stcg_1_3d()
    # plots = env_tunnel()
    plots = env_ztunnel()
    # # plots = env_zigzag()
    fig = plt.figure(figsize=(8, 8))
    ax2 = Axes3D(fig,auto_add_to_figure=False)
    fig.add_axes(ax2)
    ax2.axis('off')
    if len(trajs) <= 5:
        cmap = plt.get_cmap('tab10')
        colors = [cmap(i) for i in np.linspace(0, 0.85, len(trajs))]

    
    ax2 = visOneTraj(ax2, plots, ref_traj, color='g', line='-', linewidth=1.)
    for i in range(len(trajs)):
        ax2 = visOneTraj(ax2, plots, trajs[i], color=colors[i], line='--',linewidth=0.1)
        # ax2 = visOneTraj(ax2, plots, trajs[i], color='r', line='--',linewidth=0.1)

    # ax2.view_init(elev=14, azim=-112) # ltunnel
    ax2.view_init(elev=15, azim=-70) # ztunnel
    result = drawtube(ax2, ref_traj, 0.2, color='b')
    # ax2.view_init(elev=91, azim=-90) # zigzag
    # result = drawtube(ax2, ref_traj, 0.1, color='b')
    # ax2.view_init(elev=91, azim=-90) # stlcg


    plt.axis('on')
    ax2.set_xlabel('')
    ax2.set_ylabel('')
    ax2.set_zlabel('')
    # Create a PDF file and save the plots to it
    plt.savefig("ztunnel_ref_5track.pdf", bbox_inches='tight', pad_inches=0, dpi=300)
    plt.show()

    return trajs


if __name__ == '__main__':
    ref_file = 'ztunnel_rand_initial/ztunnel_gc_real_traj_rand0.csv'
    # track_file = 'real_traj_gc/ltunnel_gc_real_traj.csv'

    drawmultipTraj(ref_file)

    # ref_traj, track_traj = test(ref_file, track_file)
    #  # Axes3D = Axes3D  # pycharm auto import
    # # plots = env_stcg_1_3d()
    # plots = env_tunnel()
    # # plots = env_ztunnel()
    # # plots = env_zigzag()
    # fig = plt.figure(figsize=(8, 8))
    # ax2 = visOneTraj(fig, plots, ref_traj, track_traj)

    # # ax1 = fig1.add_subplot(111,projection='3d')
    # result = drawtube(ax2, ref_traj, 0.2, color='b')
    # # result = drawtube(ax2, track_traj, 0.09181, color='r')
    # # plt.axis('on')
    # # plt.legend()
    # # plt.savefig("Ztunnel_compare.pdf", dpi=600, format='pdf') ## 保存图片

    # # Set the view for z_tunnel
    # ax2.view_init(elev=20, azim=-60) # ltunnel
    # # ax2.view_init(elev=90, azim=90) # zigzag
    # # ax2.view_init(elev=91, azim=-90) # stlcg

    # plt.axis('on')
    # ax2.set_xlabel('')
    # ax2.set_ylabel('')
    # ax2.set_zlabel('')
    # # Create a PDF file and save the plots to it
    # plt.savefig("ltunnel_ref_track.pdf", bbox_inches='tight', pad_inches=0, dpi=300)
    # plt.show()



