from stl import mesh
import numpy as np
import matplotlib.pyplot as plt

# Load the STL files and add the vectors to the plot
your_mesh = mesh.Mesh.from_file('MapaGazebo.stl')

resolution = 1.0
x_size = int((your_mesh.max_[0] - your_mesh.min_[0]) / resolution)
y_size = int((your_mesh.max_[1] - your_mesh.min_[1]) / resolution)
z_size = int((your_mesh.max_[2] - your_mesh.min_[2]) / resolution)
grid_map = np.zeros((z_size, y_size, x_size))

points_to_plot = []

for triangle in your_mesh.vectors:
    for point in triangle:
        print(point)
        x_idx = int((point[0] - your_mesh.min_[0]) // resolution)
        y_idx = int((point[1] - your_mesh.min_[1]) // resolution)
        z_idx = int((point[2] - your_mesh.min_[2]) // resolution)
        print(x_idx, y_idx, z_idx)
        if x_idx < x_size and y_idx < y_size and z_idx < z_size:
            grid_map[z_idx, y_idx, x_idx] = 100
        else:
            print('The point doesnt fit on grid map!')
        points_to_plot.append((x_idx, y_idx, z_idx))


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
for p in points_to_plot:
    ax.scatter(p[0], p[1], p[2])
plt.show()
