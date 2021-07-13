#!/usr/bin/env python
import rospy as rp
from visualization_msgs.msg import Marker


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
    st = Point3D(0.5, 0.5, 0.0, "start", (0.0, 1.0, 0.0))
    en = Point3D(0.5, 1.3, 0.4, "end", (1.0, 0.0, 0.0))
    while not rp.is_shutdown():
        st.publish()
        en.publish()
        rp.sleep(0.5)
