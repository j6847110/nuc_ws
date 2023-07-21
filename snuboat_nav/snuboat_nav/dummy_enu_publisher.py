import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64 ,Float64MultiArray, MultiArrayDimension
import numpy as np

class Dummy_ENU_Publisher(Node):
    def __init__(self):
        super().__init__('dummy_enu_publisher')


        self.dt = 0.1
        self.OS_enu_x = np.linspace(0, 100, 100)
        self.OS_enu_y = np.linspace(0, 0, 100)

        self.cnt = 0

        self.OS_enu_pos_pub = self.create_publisher(Point, '/enu_pos', 1)
        self.OS_heading_pub = self.create_publisher(Float64, '/heading', 1)
        self.OS_spd_pub = self.create_publisher(Float64, '/spd', 1)

        self.timer = self.create_timer(self.dt, self.pub_enu_pos)

        self.wp_set = np.vstack(([0.,0.],[20.,0.],[40.,0.],[60.,0.]))
        self.enu_wp_set_pub = self.create_publisher(Float64MultiArray, '/enu_wp_set', 1)

        self.timer = self.create_timer(self.dt, self.pub_wp_set)

    def pub_enu_pos(self):
        OS_enu_pos_p = Point()
        OS_enu_pos_p.x = self.OS_enu_x[self.cnt]
        OS_enu_pos_p.y = self.OS_enu_y[self.cnt]
        self.OS_enu_pos_pub.publish(OS_enu_pos_p)

        OS_heading_pub_ = Float64()
        OS_spd_pub_ = Float64()
        if self.cnt != 0:

            OS_heading_pub_.data = np.arctan2(self.OS_enu_y[self.cnt] - self.OS_enu_y[self.cnt - 1], self.OS_enu_x[self.cnt] - self.OS_enu_x[self.cnt - 1])
            OS_spd_pub_.data = np.linalg.norm([self.OS_enu_x[self.cnt] - self.OS_enu_x[self.cnt - 1], self.OS_enu_y[self.cnt] - self.OS_enu_y[self.cnt - 1]],2)
        else:
            OS_heading_pub_.data = 0.
            OS_spd_pub_.data = 0.
        self.OS_heading_pub.publish(OS_heading_pub_)      
        self.OS_spd_pub.publish(OS_spd_pub_)

        self.cnt += 1
        if self.cnt == len(self.OS_enu_x):
            self.get_logger().info('Exiting dummy_enu_publisher')
        else:
            self.get_logger().info('Executing dummy_enu_publisher')
    def pub_wp_set(self):
        wp_set = Float64MultiArray()
        temp = np.zeros((len(self.wp_set),2))
        for i in range(len(self.wp_set)):
            point_x = float(self.wp_set[i][0])
            point_y = float(self.wp_set[i][1])
            temp[i]=[point_x,point_y]
        b = temp.reshape([8])
        wp_set.data = b.tolist()
        
        wp_set.layout.data_offset = 0 
        # create two dimensions in the dim array
        wp_set.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        # dim[0] is the vertical dimension of your matrix
        wp_set.layout.dim[0].label = "channels"
        wp_set.layout.dim[0].size = 2
        wp_set.layout.dim[0].stride = 2*len(self.wp_set)
        # dim[1] is the horizontal dimension of your matrix
        wp_set.layout.dim[1].label = "samples"
        wp_set.layout.dim[1].size = len(self.wp_set)
        wp_set.layout.dim[1].stride = len(self.wp_set)
        
        
        self.enu_wp_set_pub.publish(wp_set)
        self.get_logger().info('publish wp_set')

def main(args=None):
    rclpy.init(args=args)
    dummy_enu_pub = Dummy_ENU_Publisher()
    rclpy.spin(dummy_enu_pub)
    dummy_enu_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
