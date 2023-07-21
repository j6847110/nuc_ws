# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point, TwistWithCovarianceStamped
from std_msgs.msg import String, Float64MultiArray, Float32
from sensor_msgs.msg import NavSatFix
import pymap3d as pm
import numpy as np

class GNSSConverter(Node):
    def __init__(self):
        super().__init__('gnss_converter')

        # config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'gnss_config.yaml')
        # with open(config_file, 'r') as file:
        #     config = yaml.safe_load(file)
        #     self.origin = self.get_parameter('origin').value
        default_params = {
            # to be modified
            'Left_Bottom' : [37.455736, 126.95174429999999, 150.781],
            'Right_Bottom' : [37.4557862, 126.9517374, 150.800], # TODO: collect when status = 2
            'Left_Top' : [37.4557289, 126.9516639, 150.803],
            'Right_Top' : [37.4557791, 126.95165829999999,151.036],
            'origin' : [37.455736, 126.95174429999999, 150.781],
        }
        self.Left_Bottom = self.declare_parameter("Left_Bottom", default_params['Left_Bottom']).value
        self.Right_Bottom = self.declare_parameter("Right_Bottom", default_params['Right_Bottom']).value
        self.Left_Top = self.declare_parameter("Left_Top", default_params['Left_Top']).value
        self.Right_Top = self.declare_parameter("Right_Top", default_params['Right_Top']).value
        self.origin = self.declare_parameter("origin", default_params['origin']).value
        # self.temp_wp = [37.456002667,126.9513785]
        # self.wp_set = np.vstack((self.Right_Top, self.Left_Top, self.Right_Bottom,self.origin))
        self.wp_set = np.vstack((self.Left_Bottom, self.Left_Top, self.Right_Bottom, self.Right_Top))
        # print(self.wp_set)
        # print(type(self.wp_set[0,0]))

        self.pos = [0, 0, 0]

        # self.gps_lon_sub = self.create_subscription(String, '/gps/lon', self.gps_lon_callback, 1)
        # self.gps_lat_sub = self.create_subscription(String, '/gps/lat', self.gps_lat_callback, 1)

        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 1)

        self.enu_pos_pub = self.create_publisher(Point, '/enu_pos', 1)
        self.enu_wp_x_set_pub = self.create_publisher(Float64MultiArray, '/enu_wp_set/x', 1)
        self.enu_wp_y_set_pub = self.create_publisher(Float64MultiArray, '/enu_wp_set/y', 1)

        self.OS_timer = self.create_timer(0.1, self.pub_enu_pos)

        self.enu_pos_log_pub = self.create_publisher(String, '/log/enu_pos', 1)
       
        self.gps_received = False
        self.gps_spd_received = False
        self.waypoint_received = False

    def wait_for_topics(self):
        self.timer = self.create_timer(10.0, self.check_topics_status)

    def check_topics_status(self):
        if not self.gps_received:
            self.get_logger().info('No topic gps_received')
        else:
            self.get_logger().info('All topics received')

    def gps_callback(self, msg):
        self.gps_received = True
        self.gps_lon = msg.longitude
        self.gps_lat = msg.latitude
        self.gps_alt=msg.altitude
        

    def enu_convert(self, gnss):
        e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], self.origin[0], self.origin[1], self.origin[2])
        return e, n, u
    
    def pub_enu_pos(self):
        if self.gps_received:
            self.pos[0], self.pos[1], self.pos[2] = self.enu_convert([self.gps_lat, self.gps_lon,self.gps_alt])
            enu_pos = Point()
            enu_pos.x = self.pos[0]
            enu_pos.y = self.pos[1]
            self.enu_pos_pub.publish(enu_pos)
            
            enu_pos_log = String()
            enu_pos_log.data = str(self.pos[0]) + "," + str(self.pos[1])
            self.enu_pos_log_pub.publish(enu_pos_log)
            print("this is my origin")
            print(self.enu_convert([self.origin[0], self.origin[1],self.origin[2]]))
            print("this is my enu pos")
            print(self.pos[0],self.pos[1],self.pos[2])

        self.enu_wp_set = np.zeros((len(self.wp_set[:, 0]), 3))
        for i in range(len(self.wp_set[:, 0])):
            self.enu_wp_set[i, 0], self.enu_wp_set[i, 1], self.enu_wp_set[i, 2] = self.enu_convert([self.wp_set[i, 0], self.wp_set[i, 1],self.wp_set[i,2]])
            
        enu_wp_set_x = Float64MultiArray()
        temp_wp_set_x = self.enu_wp_set[:, 0]
        temp_wp_set_x = np.array(temp_wp_set_x)
        
        #numpy float error!!
        enu_wp_set_x.data = temp_wp_set_x.tolist()
        self.enu_wp_x_set_pub.publish(enu_wp_set_x)
        
        enu_wp_set_y = Float64MultiArray()
        temp_wp_set_y = self.enu_wp_set[:, 1]
        temp_wp_set_y = np.array(temp_wp_set_y)
        #numpy float error!!
        enu_wp_set_y.data = temp_wp_set_y.tolist()
        self.enu_wp_y_set_pub.publish(enu_wp_set_y)
        # print("this is wp set")
        # print(enu_wp_set_x.data,enu_wp_set_y.data)

def main(args=None):
    rclpy.init(args=args)
    gnss_converter = GNSSConverter()
    gnss_converter.wait_for_topics()
    rclpy.spin(gnss_converter)
    gnss_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 

