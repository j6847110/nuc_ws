# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist,Pose, PoseStamped, PoseWithCovarianceStamped,Vector3,Point , PoseWithCovariance
from microstrain_inertial_msgs.msg import FilterHeading
from std_msgs.msg import Bool, Int8, ColorRGBA, Float64MultiArray
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np
from visualization_msgs.msg import Marker,MarkerArray
import pymap3d as pm
import numpy as np

class Localizer(Node):
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'
    def __init__(self):
        super().__init__("localizer")
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
        
        self.dt = 0.1
        self.heading_q = []
        # cur position
        self.enu_pos = [0, 0, 0]


        
        self.wp_idx = 0
        # self.spd_sub = self.create_subscription(TwistWithCovarianceStamped,
        #                                         "/fix_velocity",self.spd_callback,1)
        #subscribe from imu
        self.heading_sub = self.create_subscription(
            Imu, "/nav/filtered_imu/data", self.imu_callback, 1)
        
        # self.enu_pos_sub = self.create_subscription(Point, '/enu_pos', self.enu_pos_callback, 1)
    
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback,1)
        self.local_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "local/pose", 1)
        self.spd_pub = self.create_publisher(Twist,"/cmd_vel",1)
        self.imu_pub = self.create_publisher(Imu,"local/imu",1)
        
        self.imu_received = False
        self.enu_pos_received = False
        self.odom_received = False     
        self.gps_received = False
        self.spd_received = False
        self.timer = self.create_timer(0.1, self.pub_local_topic)
        
    def pub_local_topic(self):
        
        if(self.gps_received == True and self.imu_received==True):
            print("publish topic")
            #enu pos
            temp_pose = Pose()
            temp_pose.position.x = self.enu_pos[0]
            temp_pose.position.y = self.enu_pos[1]
            temp_pose.position.z = self.enu_pos[2]
            temp_pose_cov = PoseWithCovariance()
            temp_pose_cov.pose=temp_pose
            temp_diag = [self.gps_cov[0],self.gps_cov[4],self.gps_cov[8],
                         self.heading_cov[0],self.heading_cov[4],self.heading_cov[8]]
            templist = np.diag(temp_diag).flatten()
            templist = templist.tolist()
            print(templist)
            temp_pose_cov.covariance=templist
            temp_pose_covst = PoseWithCovarianceStamped()
            temp_pose_covst.pose=temp_pose_cov
            temp_pose_covst.header._frame_id="odom"
            
            self.imu_pub.publish(self.imu_data)
            self.local_pose_pub.publish(temp_pose_covst)
        if(self.spd_received ==True):
            print("publish speed")
            print(self.spd)
            self.spd_pub.publish(self.spd)
    #
    def spd_callback(self,msg):
        self.spd_received = True
        self.spd = msg.twist.twist
        print(self.spd)
        print("spd ")   
    def enu_convert(self, gnss):
        e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], self.origin[0], self.origin[1], self.origin[2])
        return e, n, u
    
    def imu_callback(self,msg):
        self.imu_received = True
        self.imu_data = msg
        self.heading_q = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        self.heading_cov = msg.orientation_covariance
        print("this is cov heading")
        print(self.heading_cov)

    def gps_callback(self, msg):
        self.gps_received = True
        self.gps_lon = msg.longitude
        self.gps_lat = msg.latitude
        self.gps_alt=msg.altitude
        self.gps_cov = msg.position_covariance
        self.enu_pos[0], self.enu_pos[1], self.enu_pos[2] = self.enu_convert([self.gps_lat, self.gps_lon,self.gps_alt])
        print("this is gps cov")
        print(self.gps_cov)
            
                
def main(args=None):
    rclpy.init(args=args)
    localizer = Localizer()
    rclpy.spin(localizer)
    localizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()