from copy import copy
import rospy as rp
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import numpy as np
from points_3d import Point3D
import os


class GridMap3D(object):
    def __init__(self):

        maps_dir = '/home/radek/catkin_mapr/src/mapr_drone_project/maps'
        map_name = 'b'
        map_3d = []
        for filename in sorted(os.listdir(maps_dir)):
            if (map_name in filename) and filename.endswith('.png'):
                print(filename)
                map_layer = cv2.imread(os.path.join(maps_dir, filename), cv2.IMREAD_GRAYSCALE)
                map_3d.append(map_layer)
        map_3d = np.array(map_3d)
        self.map = (map_3d < 99).astype(int) * 100
        self.map_resolution = 0.1
        self.map_pub = rp.Publisher('markers_map', MarkerArray, queue_size=10)

        rp.init_node('graph_search')

        self.start = None
        self.end = None

        rp.Subscriber('point_start', Marker, self.set_start)
        rp.Subscriber('point_end', Marker, self.set_end)
        self.path_pub = rp.Publisher('path', Path, queue_size=10)
        while self.map is None or self.start is None or self.end is None:
            rp.sleep(0.1)
        print("Object initialized!")

    def map_callback(self, data):
        self.map = copy(data)
        self.map.data = list(self.map.data)

    def get_marker_xyz(self, marker):
        mul = 1. / self.map_resolution
        x = int(marker.pose.position.y * mul)
        y = int(marker.pose.position.x * mul)
        z = int(marker.pose.position.z * mul)
        return x, y, z

    def set_start(self, data):
        x, y, z = self.get_marker_xyz(data)
        self.start = (x, y, z)

    def set_end(self, data):
        x, y, z = self.get_marker_xyz(data)
        self.end = (x, y, z)

    def publish_visited(self):
        self.pub.publish(self.map)
        rp.sleep(0.05)

    def publish_map_as_points(self):
        walls_indicies = np.argwhere(self.map == 100)
        markers_array = MarkerArray()
        for idx in walls_indicies:
            x_pos = idx[1] * self.map_resolution
            y_pos = idx[2] * self.map_resolution
            z_pos = idx[0] * self.map_resolution
            pt = Point3D(x_pos, y_pos, z_pos, f"{idx[1]}_{idx[2]}_{idx[0]}", (0.3, 0.5, 0.5))
            markers_array.markers.append(pt.marker)
        self.map_pub.publish(markers_array)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for p in path:
            rp.logerr(p)
            pose = PoseStamped()
            pose.pose.position.x = self.map_resolution * p[1] + 0.05
            pose.pose.position.y = self.map_resolution * p[0] + 0.05
            pose.pose.position.z = self.map_resolution * p[2] + 0.05
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            pose.header.frame_id = 'map'
            pose.header.stamp = rp.Time.now()
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def search(self):
        return NotImplementedError()
