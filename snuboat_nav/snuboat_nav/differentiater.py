# SNUBOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, String
import numpy as np

class Differentiater(Node):
    def __init__(self):
        super().__init__('differentiater')

        # config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'differentiater_config.yaml')
        # with open(config_file, 'r') as file:
        #     config = yaml.safe_load(file)
        #     self.dt = self.get_parameter('dt').value
        default_params = {
            'dt' : 0.1
        }
        self.dt = self.declare_parameter("dt", default_params['dt']).value

        self.OS_enu_pos = np.zeros((10, 2))

        self.OS_spd = np.zeros(10)
        self.OS_heading = np.zeros(10)

        self.OS_enu_pos_sub = self.create_subscription(Point, '/enu_pos', self.OS_enu_pos_callback, 1)

        self.OS_spd_pub = self.create_publisher(Float64, '/spd', 1)
        self.OS_heading_pub = self.create_publisher(Float64, '/heading', 1)

        # for log 
        self.OS_spd_log_pub = self.create_publisher(String, '/log/spd', 1)
        self.OS_heading_log_pub = self.create_publisher(String, '/log/heading', 1)

        self.OS_timer = self.create_timer(self.dt, self.OS_pub)

        self.OS_enu_pos_received = False

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topics_status)

    def check_topics_status(self):
        if not self.OS_enu_pos_received:
            self.get_logger().info('No topic OS_enu_pos_received')
        if self.OS_enu_pos_received:
            self.get_logger().info('All topics received')
        else:
            self.get_logger().info('Waiting for topics to be published...')


    def OS_enu_pos_callback(self, msg):
        self.OS_enu_pos_received = True
        self.OS_enu_pos = np.append(self.OS_enu_pos, [[msg.x, msg.y]], axis=0)
        self.OS_enu_pos = self.OS_enu_pos[1:]

    def OS_pub(self):
        # delay minimum 10*dt
        if self.OS_enu_pos_received:
            if (self.OS_enu_pos[-1, 0] != self.OS_enu_pos[-2, 0]) or (self.OS_enu_pos[-1, 1] != self.OS_enu_pos[-2, 1]):
                cur_OS_spd = np.linalg.norm(self.OS_enu_pos[-1, :] - self.OS_enu_pos[-2, :]) / self.dt
                cur_OS_heading = np.arctan2(self.OS_enu_pos[-1, 1] - self.OS_enu_pos[-2, 1], self.OS_enu_pos[-1, 0] - self.OS_enu_pos[-2, 0])
                cur_OS_heading = np.degrees(cur_OS_heading)
            else:
                cur_OS_spd = self.OS_spd[-1]
                cur_OS_heading = self.OS_heading[-1]
            
            self.OS_spd = np.append(self.OS_spd, cur_OS_spd)
            self.OS_spd = self.OS_spd[1:]

            self.OS_heading = np.append(self.OS_heading, cur_OS_heading)
            self.OS_heading = self.OS_heading[1:]

            temp_spd = Float64()
            temp_spd.data = cur_OS_spd

            temp_heading = Float64()
            temp_heading.data = cur_OS_heading

            self.OS_spd_pub.publish(temp_spd)
            self.OS_heading_pub.publish(temp_heading)

            # publish for log data
            temp_spd_log = String()
            temp_heading_log = String()
            temp_spd_log.data = str(cur_OS_spd)
            temp_heading_log.data = str(cur_OS_heading)
            self.OS_spd_log_pub.publish(temp_spd_log)
            self.OS_heading_log_pub.publish(temp_heading_log)

def main(args=None):
    rclpy.init(args=args)
    differentiater = Differentiater()
    differentiater.wait_for_topics()
    rclpy.spin(differentiater)
    differentiater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
