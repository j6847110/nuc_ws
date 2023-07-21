# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from microstrain_inertial_msgs.msg import FilterHeading
from std_msgs.msg import ColorRGBA, Bool, Int8, Int32, Float32, Float64, String, Float64MultiArray, Int32MultiArray
import numpy as np
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import rviz_visualizer as visual

class RVIZ_visual(Node):
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'
    def __init__(self):
        super().__init__("rviz_pub")
        self.dt = 0.1

        self.viz_marker_pub = self.create_publisher(Marker,"viz/marker",1)

        
        self.timer = self.create_timer(self.dt, self.viz_pub)
    def viz_pub(self):
        # Create a marker message
        marker = Marker()
        # Set the necessary properties of the marker
        marker.header.frame_id = "map"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0+self.count/100
        marker.pose.position.y = 1.0
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.viz_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    rviz_visual = RVIZ_visual()
    rclpy.spin(rviz_visual)
    rviz_visual.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
