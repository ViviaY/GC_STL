
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import Akima1DInterpolator

def tube_path(points, radius):
    # 使用Catmull-Rom样条插值平滑地连接这些点
    t = np.arange(len(points))
    cs = Akima1DInterpolator(t, points) #, bc_type='clamped')

    # 插值点数量
    num_interpolation_points = 200
    t_new = np.linspace(t.min(), t.max(), num_interpolation_points)
    points_smooth = cs(t_new)

    # 计算管道段的向量
    vectors = np.diff(points_smooth, axis=0)

    # 计算管道段的顶点
    num_points = 20
    angles = np.linspace(0, 2 * np.pi, num_points)

    X, Y, Z = [], [], []
    for i, vec in enumerate(vectors):
        start = points_smooth[i]
        end = points_smooth[i + 1]
        length = np.linalg.norm(vec)
        vec = vec / length

        # 计算管道段的轴
        z_axis = np.cross(vec, np.array([0, 0, 1]))
        y_axis = np.cross(z_axis, vec)
        z_axis = z_axis / np.linalg.norm(z_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # 计算管道段的顶点
        for j, angle in enumerate(angles):
            direction = np.cos(angle) * y_axis + np.sin(angle) * z_axis
            x, y, z = start + radius * direction
            X.append(x)
            Y.append(y)
            Z.append(z)

    # 使用 plot_trisurf 绘制隧道
    tri_indices = []
    for i in range(num_interpolation_points - 2):
        for j in range(num_points):
            tri_indices.append([i * num_points + j,
                                i * num_points + (j + 1) % num_points,
                                (i + 1) * num_points + j])
            tri_indices.append([i * num_points + (j + 1) % num_points,
                                (i + 1) * num_points + (j + 1) % num_points,
                                (i + 1) * num_points + j])

    return X, Y, Z, tri_indices


def drawtube(ax, traj,radius,color='r'):

    
    points = np.array([traj[i][1] for i in range(len(traj))])

    X, Y, Z, tri_indices = tube_path(points, radius)
    # ax.axis('on')
    ax.plot_trisurf(X, Y, Z, triangles=tri_indices, alpha=0.1, color=color, label="robustness tube")  

    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # 设置轴的缩放比例
    # ax.auto_scale_xyz([-0.7, 0.7], [-0.7, 0.7], [0, 1])   



# if __name__ == '__main__':
#     x = [1, 2, 3, 4, 5]
#     y = [2, 3, 4, 5, 6]
#     z = [3, 4, 5, 6, 7]
#     radius = 0.5
#     traj = list[x,y,z]
    # drawtube(traj,radius)



