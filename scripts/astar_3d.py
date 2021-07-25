#!/usr/bin/env python
import time

import rospy as rp
from grid_map_3d import GridMap3D
from points_3d import Point3D
import heapq as pq
import math
from scipy.interpolate import LSQUnivariateSpline
import numpy as np


class AStar3D(GridMap3D):
    def __init__(self):
        super(AStar3D, self).__init__()

        self.directions = [(0, -1, 0), (-1, 0, 0), (0, 1, 0), (1, 0, 0), (0, 0, -1), (0, 0, 1)]

    def heuristics(self, pos):
        # Euclidean
        distance = math.sqrt((pos[0] - self.end[0])**2 + (pos[1] - self.end[1])**2 + (pos[2] - self.end[2])**2)

        return distance

    def search(self):

        # Cells to visit single element -> (cost=distance+heuristics, point=(x, y, z))
        cells_to_visit = [(0, self.start)]
        distances = {str((i, j, k)): np.inf
                     for i in range(self.map.shape[2])
                     for j in range(self.map.shape[1])
                     for k in range(self.map.shape[0])}
        distances[str(self.start)] = 0
        parents = {}

        while cells_to_visit:

            current_cell = pq.heappop(cells_to_visit)[1]

            # Check if the current cell is the end cell
            if current_cell == self.end:
                print("End cell found!")
                break

            self.publish_map_as_points()
            # Check if the cell is not a wall
            if not (self.map[current_cell[2], current_cell[1], current_cell[0]] == 100):
                # Mark as visited
                self.map.data[current_cell[2], current_cell[1], current_cell[0]] = 50

                # Iterate over neighbours
                for d in self.directions:
                    neighbour = (current_cell[0] + d[0], current_cell[1] + d[1], current_cell[2] + d[2])

                    # Check if the neighbours exists on the map
                    if (neighbour[0] >= 0) and (neighbour[1] >= 0) and (neighbour[2] >= 0)\
                            and (neighbour[0] < self.map.shape[2]) and (neighbour[1] < self.map.shape[1]) and (neighbour[2] < self.map.shape[0]):

                        # print(f"old distance: {distances[str(neighbour)]}, new: {distances[str(current_cell)] + 1}")

                        # Check if the distance from the start to this neighbour is smaller than the previous one
                        if (distances[str(current_cell)] + 1) < distances[str(neighbour)]:
                            # Add this neighbour to cells to visit list
                            cost = (distances[str(current_cell)] + 1) + self.heuristics(neighbour)
                            pq.heappush(cells_to_visit, (cost, neighbour))

                            # Update distance
                            distances[str(neighbour)] = distances[str(current_cell)] + 1

                            # Make current cell as a parent of this neighbour
                            parents[str(neighbour)] = current_cell

            # Every iteration publishes visited cells
            # self.publish_visited()

        # When the end cell is found, calculate a path to it
        backtrace = [self.end]
        while True:
            parent = parents[str(backtrace[0])]
            backtrace.insert(0, parent)
            if parent == self.start:
                break

        knots_coeff = 2
        upsampling = 10
        backtrace_np = np.array(backtrace)
        smooth_backtrace_np = np.zeros((backtrace_np.shape[0] * upsampling, backtrace_np.shape[1]))
        for i in range(backtrace_np.shape[1]):
            y = backtrace_np[:, i]
            x = np.linspace(0, backtrace_np.shape[0], backtrace_np.shape[0])
            t = np.linspace(0, backtrace_np.shape[0], backtrace_np.shape[0] // knots_coeff)[1:-2]
            spl = LSQUnivariateSpline(x, y, t)
            smooth_backtrace_np[:, i] = spl(np.linspace(0, backtrace_np.shape[0], backtrace_np.shape[0] * upsampling))

        # Publish the path
        self.publish_path(smooth_backtrace_np)

        # Visualize drone's flight
        print(smooth_backtrace_np.shape)
        for point_on_path in smooth_backtrace_np:
            drone = Point3D(point_on_path[1] * self.map_resolution,
                            point_on_path[0] * self.map_resolution,
                            point_on_path[2] * self.map_resolution,
                            "drone",
                            (1.0, 1.0, 1.0))
            drone.publish()
            rp.sleep(0.1)
            print("published!")


if __name__ == '__main__':
    astar_3d = AStar3D()
    astar_3d.search()
