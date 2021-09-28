#!/usr/bin/env python
import numpy as np
import rospy as rp
from visualization_msgs.msg import Marker
import os
import json


class Point3D:
    def __init__(self, x, y, z, name, color):
        self.pub = rp.Publisher('point_' + name, Marker, queue_size=10)
        self.marker = Marker()

        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rp.Time.now()
        self.marker.ns = name
        self.marker.id = 0
        self.marker.type = Marker.CUBE
        # self.marker.action = Marker.ADD
        self.marker.pose.position.x = x + 0.05
        self.marker.pose.position.y = y + 0.05
        self.marker.pose.position.z = z + 0.05

        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1

        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = 0.5

    def publish(self):
        self.pub.publish(self.marker)


if __name__ == '__main__':
    rp.init_node('points_3d')

    map_resolution = 0.1
    maps_dir = '/home/radek/catkin_mapr/src/mapr_drone_project/maps'
    json_filename = 'B.json'
    with open(os.path.join(maps_dir, json_filename)) as f:
        points_json = json.load(f)
        start_pt = np.array(points_json['start']) * map_resolution
        end_pt = np.array(points_json['end']) * map_resolution

    st = Point3D(start_pt[1], start_pt[0], start_pt[2], "start", (0.0, 1.0, 0.0))
    en = Point3D(end_pt[1], end_pt[0], end_pt[2], "end", (1.0, 0.0, 0.0))
    while not rp.is_shutdown():
        st.publish()
        en.publish()
        rp.sleep(0.5)
