import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

# Create the 3D block triangle points
def create_3d_block_triangle_points(size):
    points = []
    for i in range(size):
        for j in range(size - i):
            for k in range(size):
                points.append([i, j, k])
    return np.array(points)

# Set the size of the 3D block triangle
size = 5

# Create the 3D block triangle points
block_triangle_points = create_3d_block_triangle_points(size)

# Compute the Convex Hull
hull = ConvexHull(block_triangle_points)

# Set up the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# # Plot the 3D block triangle points
# ax.scatter(block_triangle_points[:, 0], block_triangle_points[:, 1], block_triangle_points[:, 2], marker='o')

# Plot the Convex Hull
for s in hull.simplices:
    # s = np.append(s, s[0])  # Here we cycle back to the first vertex in the simplex to close the polygon
    ax.plot(block_triangle_points[s, 0], block_triangle_points[s, 1], block_triangle_points[s, 2], "k-")

ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
plt.title('3D Block Triangle with Convex Hull')

# Show the plot
plt.show()
