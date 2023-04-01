from gurobipy import *
import numpy as np
from collections import namedtuple
import time
import pickle
import pypoman as ppm
import matplotlib.pyplot as plt; 
import matplotlib.path as mpath
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull


def plot_line_cubion(ax, vertices, color='red'):

    vertices = np.concatenate( np.array([vertices],dtype=object), axis=0)
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

def plot_line_Triangle(ax, vertices, color='red'):
    hull = ConvexHull(vertices)
    points = []
    for i in range(len(vertices)):
        points.append([vertices[i][0], vertices[i][1],vertices[i][2]])
    p = np.array(points)
    vertices = np.array([vertices])
    # Plot the surface of the polyhedron
    for s in hull.simplices:
        # s = np.append(s, s[0])  # Here we cycle back to the first vertex in the simplex to close the polygon
        ax.plot(p[s, 0], p[s, 1], p[s, 2], color=color)
        # ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles=faces, shade=True)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    # plt.title('3D Block Triangle with Convex Hull')


def vis(test, limits=None, equal_aspect=True):
    _, plots, PWLs = test()

    # print(PWLs)
    plt.rcParams["figure.figsize"] = [6.4, 6.4]
    plt.rcParams['axes.titlesize'] = 20
    fig = plt.figure()
    ax = Axes3D(fig,auto_add_to_figure=False)
    fig.add_axes(ax)
    ax.axis('off')

    vertices = []
    for plot in plots:
        for A, b in plot[0]:
            vs = ppm.duality.compute_polytope_vertices(A, b) 
            if len(plot) == 3:
                if plot[2] == 'tri':
                    plot_line_Triangle(ax, vs, color=plot[1])
                    vertices.append(vs)
            else:
                plot_line_cubion(ax, vs, color=plot[1])
                vertices.append(vs)
                # ax.add_collection3d(Poly3DCollection(vertices))
                # ppm.polygon.plot_polygon(vs, color = plot[1], alpha=1.)

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
        plt.gca().set_aspect('equal', adjustable='box')

    if PWLs is None or PWLs[0] is None:
        plt.show()
        return

    if len(PWLs) <= 4:
        colors = ['k', np.array([153,0,71],dtype=object)/255, np.array([6,0,153],dtype=object)/255, np.array([0, 150, 0],dtype=object)/255]
    else:
        cmap = plt.get_cmap('tab10')
        colors = [cmap(i) for i in np.linspace(0, 0.85, len(PWLs))]


    ax.plot([PWLs[i][0][0] for i in range(len(PWLs))], [PWLs[i][0][1] for i in range(len(PWLs))], [PWLs[i][0][2] for i in range(len(PWLs))],'.-', color = colors[1])
    
    ax.plot(PWLs[-1][0][0], PWLs[-1][0][1], PWLs[-1][0][2], '*', color = 'g')
    ax.plot(PWLs[0][0][0], PWLs[0][0][1], PWLs[0][0][2], 'o', color = 'g')

    plt.show()



def visTwoTraj(ax, plots, refTraj, trackTraj, limits=None, equal_aspect=True):

    # print(PWLs)
    plt.rcParams["figure.figsize"] = [6.4, 6.4]
    plt.rcParams['axes.titlesize'] = 20
    # fig = fig #plt.figure()
    # ax = Axes3D(fig,auto_add_to_figure=False)
    # fig.add_axes(ax)
    # ax.axis('off')

    vertices = []
    for plot in plots:
        for A, b in plot[0]:
            vs = ppm.duality.compute_polytope_vertices(A, b) 
            if len(plot) == 3:
                if plot[2] == 'tri':
                    plot_line_Triangle(ax, vs, color=plot[1])
                    vertices.append(vs)
            else:
                plot_line_cubion(ax, vs, color=plot[1])
                vertices.append(vs)
                # ax.add_collection3d(Poly3DCollection(vertices))
                # ppm.polygon.plot_polygon(vs, color = plot[1], alpha=1.)

    if limits is not None:
        ax.xlim(limits[0])
        ax.ylim(limits[1])
        ax.zlim(limits[2])
    else:
        vertices = np.concatenate(vertices, axis=0)
        xmin, ymin, zmin = vertices.min(axis=0)
        xmax, ymax, zmax = vertices.max(axis=0)
        ax.set_xlim([xmin - 0.5, xmax + 0.5])
        ax.set_ylim([ymin - 0.5, ymax + 0.5])
        ax.set_zlim([zmin - 0.5, zmax + 0.5])

    if equal_aspect:
        plt.gca().set_aspect('equal', adjustable='box')

    if refTraj is None or trackTraj is None:
        plt.show()
        return



    ax.plot([refTraj[i][1][0] for i in range(len(refTraj))], [refTraj[i][1][1] for i in range(len(refTraj))], [refTraj[i][1][2] for i in range(len(refTraj))],
            '-', color = 'g', label="reference trajectory")
    ax.plot(refTraj[-1][1][0], refTraj[-1][1][1], refTraj[-1][1][2], '-', color = 'g')
    ax.plot(refTraj[0][1][0], refTraj[0][1][1], refTraj[0][1][2], '-', color = 'g')

    ax.plot([trackTraj[i][1][0] for i in range(len(trackTraj))], [trackTraj[i][1][1] for i in range(len(trackTraj))], [trackTraj[i][1][2] for i in range(len(trackTraj))],
            '--', color = 'r',label="tracking trajectory")
    ax.plot(trackTraj[-1][1][0], trackTraj[-1][1][1], trackTraj[-1][1][2], '--', color = 'r')
    ax.plot(trackTraj[0][1][0], trackTraj[0][1][1], trackTraj[0][1][2], '--', color = 'r')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([]) 
    # plt.show()
    return ax


def visOneTraj(ax, plots, trackTraj, color='g', line='--',  linewidth=2.0, limits=None, equal_aspect=True):

    # print(PWLs)
    plt.rcParams["figure.figsize"] = [6.4, 6.4]
    plt.rcParams['axes.titlesize'] = 20
    # fig = fig #plt.figure()
    # ax = Axes3D(fig,auto_add_to_figure=False)
    # fig.add_axes(ax)
    # ax.axis('off')

    vertices = []
    for plot in plots:
        for A, b in plot[0]:
            vs = ppm.duality.compute_polytope_vertices(A, b) 
            if len(plot) == 3:
                if plot[2] == 'tri':
                    plot_line_Triangle(ax, vs, color=plot[1])
                    vertices.append(vs)
            else:
                plot_line_cubion(ax, vs, color=plot[1])
                vertices.append(vs)
                # ax.add_collection3d(Poly3DCollection(vertices))
                # ppm.polygon.plot_polygon(vs, color = plot[1], alpha=1.)

    if limits is not None:
        ax.xlim(limits[0])
        ax.ylim(limits[1])
        ax.zlim(limits[2])
    else:
        vertices = np.concatenate(vertices, axis=0)
        xmin, ymin, zmin = vertices.min(axis=0)
        xmax, ymax, zmax = vertices.max(axis=0)
        ax.set_xlim([xmin - 0.5, xmax + 0.5])
        ax.set_ylim([ymin - 0.5, ymax + 0.5])
        ax.set_zlim([zmin - 0.5, zmax + 0.5])

    if equal_aspect:
        plt.gca().set_aspect('equal', adjustable='box')


    ax.plot([trackTraj[i][1][0] for i in range(len(trackTraj))], [trackTraj[i][1][1] for i in range(len(trackTraj))], [trackTraj[i][1][2] for i in range(len(trackTraj))],
            line, color =color)
    ax.plot(trackTraj[-1][1][0], trackTraj[-1][1][1], trackTraj[-1][1][2], line, color = color, linewidth=linewidth)
    ax.plot(trackTraj[0][1][0], trackTraj[0][1][1], trackTraj[0][1][2], line, color = color, linewidth=linewidth)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([]) 
    # plt.show()
    return ax

def vis_Robust(plots, refTraj, rho, limits=None, equal_aspect=True):

    # print(PWLs)
    plt.rcParams["figure.figsize"] = [6.4, 6.4]
    plt.rcParams['axes.titlesize'] = 20
    fig = plt.figure()
    ax = Axes3D(fig,auto_add_to_figure=False)
    fig.add_axes(ax)
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
        plt.gca().set_aspect('equal', adjustable='box')

    if refTraj is None:
        plt.show()
        return

    if len(refTraj) <= 4:
        colors = ['k', np.array([153,0,71],dtype=object)/255, np.array([6,0,153],dtype=object)/255, np.array([0, 150, 0],dtype=object)/255]
    else:
        cmap = plt.get_cmap('tab10')
        colors = [cmap(i) for i in np.linspace(0, 0.85, len(refTraj))]


    ax.plot([refTraj[i][1][0] for i in range(len(refTraj))], [refTraj[i][1][1] for i in range(len(refTraj))], [refTraj[i][1][2] for i in range(len(refTraj))],'.-', color = 'g')
    ax.plot(refTraj[-1][1][0], refTraj[-1][1][1], refTraj[-1][1][2], '*', color = 'g')
    ax.plot(refTraj[0][1][0], refTraj[0][1][1], refTraj[0][1][2], 'o', color = 'g')

    plt.show()


