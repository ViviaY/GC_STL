from gurobipy import *
import numpy as np
from collections import namedtuple
import time
import pickle
import pypoman as ppm
import matplotlib.pyplot as plt; plt.rcdefaults()
import matplotlib.path as mpath
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from math import cos, sin
from Quadrotor import Quadrotor

def plot_line_cubion(ax, vertices, color='red'):

    vertices = np.concatenate( np.array([vertices]), axis=0)
    x, y, z = vertices.min(axis=0)
    dx, dy, dz = vertices.max(axis=0) - vertices.min(axis=0)
    xx = [x, x, x+dx, x+dx, x]
    yy = [y, y+dy, y+dy, y, y]
    kwargs = {'alpha': 1, 'color': color}
    ax.plot3D(xx, yy, [z]*5, **kwargs)
    ax.plot3D(xx, yy, [z+dz]*5, **kwargs)
    ax.plot3D([x, x], [y, y], [z, z+dz], **kwargs)
    ax.plot3D([x, x], [y+dy, y+dy], [z, z+dz], **kwargs)
    ax.plot3D([x+dx, x+dx], [y+dy, y+dy], [z, z+dz], **kwargs)
    ax.plot3D([x+dx, x+dx], [y, y], [z, z+dz], **kwargs)
    # plt.title('Cube')
    # plt.show()



def vis(test, drone_size=0.1, limits=None, equal_aspect=True):
    _, plots, PWLs = test()

    print(PWLs)
    plt.rcParams["figure.figsize"] = [6.4, 6.4]
    plt.rcParams['axes.titlesize'] = 20
    plt.ion() # for showing a drone
    fig = plt.figure()
    ax = Axes3D(fig,auto_add_to_figure=False)
    fig.add_axes(ax)

    # ax = fig.add_subplot(111)
    # for stopping simulation with the esc key.
    fig.canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
    ax.axis('off')

    vertices = []
    for plot in plots:
        for A, b in plot[0]:
            vs = ppm.duality.compute_polytope_vertices(A, b)
            plot_line_cubion(ax, vs, color=plot[1])
            vertices.append(vs)
            # ax.add_collection3d(Poly3DCollection(vertices))
            # # ppm.polygon.plot_polygon(vs, color = plot[1], alpha=1.)
    

    if limits is not None:
        ax.xlim(limits[0])
        ax.ylim(limits[1])
        ax.zlim(limits[2])
    else:
        vertices = np.concatenate(vertices, axis=0)
        xmin, ymin, zmin = vertices.min(axis=0)
        xmax, ymax, zmax = vertices.max(axis=0)
        ax.set_xlim([xmin - 0.1, xmax + 0.1])
        ax.set_ylim([ymin - 0.1, ymax + 0.1])
        ax.set_zlim([zmin - 0.1, zmax + 0.1])

    if equal_aspect:
        plt.gca().set_aspect('auto', adjustable='box')

    if PWLs is None or PWLs[0] is None:
        plt.show()
        return

    if len(PWLs) <= 4:
        colors = ['k', np.array([153,0,71])/255, np.array([6,0,153])/255, np.array([0, 150, 0])/255]
    else:
        cmap = plt.get_cmap('tab10')
        colors = [cmap(i) for i in np.linspace(0, 0.85, len(PWLs))]


    # for i in range(len(PWLs)):
    #     PWL = PWLs[i]
    ax.plot(PWLs[-1][0][0], PWLs[-1][0][1], PWLs[-1][0][2], '*', color = 'r')
    ax.plot(PWLs[0][0][0], PWLs[0][0][1], PWLs[0][0][2], 'o', color = 'g')
    # ax.plot([P[0][0]for P in PWL], [P[0][1]for P in PWL], [P[0][2]for P in PWL] ,'.-', color = colors[i])
    # draw a drone
    q = Quadrotor(x=PWLs[0][0][0], y=PWLs[0][0][1], z=PWLs[0][0][2], roll=0,
                pitch=0, yaw=0, size=drone_size, ax=ax, show_animation=True)
    for P in PWLs:
        # q.update_pose(P[0][0], P[0][1], P[0][2],P[1][2],P[1][1],P[1][0])
        q.update_pose(P[0][0], P[0][1], P[0][2], 0, 0,0)
        


    plt.ioff()
    plt.show()
