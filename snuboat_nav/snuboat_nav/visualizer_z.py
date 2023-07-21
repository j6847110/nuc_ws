# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped,Vector3,Point , PoseWithCovariance
from microstrain_inertial_msgs.msg import FilterHeading
from std_msgs.msg import Bool, Int8, ColorRGBA, Float64MultiArray, Float64, Int32MultiArray
from sensor_msgs.msg import Imu
import numpy as np
from visualization_msgs.msg import Marker,MarkerArray
import random
import math
import random

def generate_random_rgba():
    red = random.randint(0, 255)/255
    green = random.randint(0, 255)/255
    blue = random.randint(0, 255)/255
    alpha = 1.0
    return [red, green, blue, alpha]

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    w = cy * cr * cp + sy * sr * sp
    x = cy * sr * cp - sy * cr * sp
    y = cy * cr * sp + sy * sr * cp
    z = sy * cr * cp - cy * sr * sp

    return [x, y, z, w]
def delete_values(array, values_to_delete):
    indices = np.isin(array, values_to_delete, invert=True)
    array = np.delete(array, np.where(indices))
    return array


class Visualizer(Node):
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'
    def __init__(self):
        super().__init__("visualizer")
        self.dt = 0.1
        self.heading_q = []
        self.enu_pos = []
        self.enu_wp_x = 0.0
        self.enu_wp_x_set = []
        self.enu_wp_y = 0.0
        self.enu_wp_y_set = []

        self.wp_idx = 0
    #subscribe from imu
        # self.heading_sub = self.create_subscription(
        #     Imu, "/nav/filtered_imu/data", self.imu_callback, 1)
        self.heading_obs_sub = self.create_subscription(
            Float64, "/nav/filtered_imu/data", self.imu_callback, 1)
        
        self.des_heading_sub = self.create_subscription(
            Float64, "/des_heading",self.des_heading_callback,1
        )
        self.ref_heading_sub = self.create_subscription(
            Float64, "/ref_heading",self.ref_heading_callback,1
        )

        #subscribe from gps
        # self.enu_pos_sub = self.create_subscription(
        #     Point, "/enu_pos", self.enu_pos_callback, 1
        # )
        # self.enu_wp_x_set_sub = self.create_subscription(
        #     Float64MultiArray, "/enu_wp_set/x", self.enu_wp_x_set_callback, 1
        # )
        # self.enu_wp_y_set_sub = self.create_subscription(
        #     Float64MultiArray, "/enu_wp_set/y", self.enu_wp_y_set_callback, 1
        # )
        # self.cur_wp_idx_sub = self.create_subscription(Int8,"/wp_idx",self.wp_idx_callback,1)


        self.viz_pose_pub = self.create_publisher(Marker, "/viz/pose", 1)
        self.viz_wp1_pub = self.create_publisher(Marker, "/viz/wp1", 1)
        self.viz_wp2_pub = self.create_publisher(Marker, "/viz/wp2", 1)
        self.viz_wp3_pub = self.create_publisher(Marker, "/viz/wp3", 1)
        self.viz_wp4_pub = self.create_publisher(Marker, "/viz/wp4", 1)
        self.viz_des_heading_pub = self.create_publisher(Marker, "/viz/des_heading", 1)
        self.viz_ref_heading_pub = self.create_publisher(Marker,"/viz/ref_heading",1)
        self.obs_phi_sub = self.create_subscription(
            Float64MultiArray, "/obs/phi", self.obs_phi_callback, 1
        )
        self.safe_heading_sub = self.create_subscription(
            Float64MultiArray, "/safe_heading", self.safe_heading_callback, 1
        )
        self.obs_labels_sub = self.create_subscription(
            Int32MultiArray, "/obs/labels", self.obs_labels_callback, 1
        )
        self.timer = self.create_timer(self.dt, self.viz_pub)
        self.viz_obs_phi_pub = self.create_publisher(MarkerArray, "/viz/obs/phi", 1)
        self.viz_safe_heading_pub = self.create_publisher(MarkerArray,"/viz/safe_heading",1)
        self.viz_danger_heading_pub = self.create_publisher(MarkerArray,"/viz/danger_heading",1)
        self.obs_phi = []
        self.obs_r=[]
        self.des_heading = 0
        self.ref_heading=0
        self.safe_heading = []
        self.des_heading_received = False
        self.imu_received = False
        self.enu_pos_received = False
        self.enu_wp_x_set_received = False
        self.enu_wp_y_set_received = False       
        self.obs_phi_received=False
        self.ref_heading_received=False
        self.color_array = [generate_random_rgba() for _ in range(100)]
        self.safe_heading_received = False


    def safe_heading_callback(self,msg):
        self.safe_heading_received = True
        self.safe_heading = msg.data

    def des_heading_callback(self,msg):
        self.des_heading_received = True
        self.des_heading = msg.data

    def ref_heading_callback(self,msg):
        self.ref_heading_received = True
        self.ref_heading = msg.data


    def obs_labels_callback(self, msg):
        self.obs_labels_received = True
        self.obs_labels = np.array(msg.data)

    def obs_phi_callback(self, msg):
        self.obs_phi_received = True
        self.obs_phi = np.array(msg.data)
        # print(self.obs_phi)
    def viz_pub(self):
        # print("visualize obs")
        cnt=5

        if(self.obs_phi_received==True and self.obs_labels_received==True and
           len(self.obs_labels)==len(self.obs_phi)):
            marker_array_msg = MarkerArray()
            #initial value

            for i in range(len(self.obs_phi)-1):
                print(len(self.obs_phi))
                print(len(self.obs_labels))
                curphi = self.obs_phi[i]
                
                curlabel=self.obs_labels[i]
                color = self.color_array[curlabel]
                curphi_q = euler_to_quaternion(0,0,curphi)
                marker = Marker()
                marker.header.frame_id = 'odom'
                marker.id = cnt
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.pose.position.x = 0.0
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = curphi_q[0]
                marker.pose.orientation.y = curphi_q[1]
                marker.pose.orientation.z = curphi_q[2]
                marker.pose.orientation.w = curphi_q[3]
                marker.scale.x = 0.5
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 1.0
                
                marker_array_msg.markers.append(marker)
                cnt+=1
            print("publish obs")
            self.viz_obs_phi_pub.publish(marker_array_msg)
        if(self.safe_heading_received==True):
            marker_array_msg = MarkerArray()
            #initial value

            for i in range(len(self.safe_heading)):
                curphi = self.safe_heading[i]
                color = [ 0.0,1.0,0.0]
                curphi_q = euler_to_quaternion(0,0,curphi)
                marker = Marker()
                marker.header.frame_id = 'odom'
                marker.id = cnt
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.pose.position.x = 0.0
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = curphi_q[0]
                marker.pose.orientation.y = curphi_q[1]
                marker.pose.orientation.z = curphi_q[2]
                marker.pose.orientation.w = curphi_q[3]
                marker.scale.x = 0.5
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 1.0
                marker_array_msg.markers.append(marker)
                cnt+=1
            print("publish safe heading")
            temp_ang = np.linspace(-np.pi, np.pi, 360).transpose()

            self.danger_heading = delete_values(temp_ang,self.safe_heading)
                #initial value
            if(len(self.danger_heading)!=0):

                for i in range(len(self.danger_heading)):
                    curphi = self.danger_heading[i]
                    color = [ 1.0,1.0,1.0]
                    curphi_q = euler_to_quaternion(0,0,curphi)
                    marker = Marker()
                    marker.header.frame_id = 'odom'
                    marker.id = cnt
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    marker.pose.position.x = 0.0
                    marker.pose.position.y = 0.0
                    marker.pose.position.z = 0.0
                    marker.pose.orientation.x = curphi_q[0]
                    marker.pose.orientation.y = curphi_q[1]
                    marker.pose.orientation.z = curphi_q[2]
                    marker.pose.orientation.w = curphi_q[3]
                    marker.scale.x = 0.5
                    marker.scale.y = 0.02
                    marker.scale.z = 0.02
                    marker.color.r = color[0]
                    marker.color.g = color[1]
                    marker.color.b = color[2]
                    marker.color.a = 1.0
                    marker_array_msg.markers.append(marker)
                    cnt+=1
                print("publish danger heading")
            self.viz_safe_heading_pub.publish(marker_array_msg)

        if(self.des_heading_received==True):
            # Create a marker message
            marker = Marker()
            curdes = self.des_heading
            curdes_q = euler_to_quaternion(0,0,curdes)
            # Set the necessary properties of the marker
            marker.header.frame_id = "odom"
            marker.id = 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = curdes_q[0]
            marker.pose.orientation.y = curdes_q[1]
            marker.pose.orientation.z = curdes_q[2]
            marker.pose.orientation.w = curdes_q[3]
            marker.scale.x = 1.0
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            print("publish my des_heading")
            self.viz_des_heading_pub.publish(marker)

        if(self.ref_heading_received==True):
            # Create a marker message
            marker = Marker()
            curref = self.ref_heading
            curref_q = euler_to_quaternion(0,0,curref)
            # Set the necessary properties of the marker
            marker.header.frame_id = "odom"
            marker.id = 2
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = curref_q[0]
            marker.pose.orientation.y = curref_q[1]
            marker.pose.orientation.z = curref_q[2]
            marker.pose.orientation.w = curref_q[3]
            marker.scale.x = 0.8
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            print("publish my ref_heading")
            self.viz_ref_heading_pub.publish(marker)

    def imu_callback(self,msg):
        self.imu_received = True
        self.heading_q = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        

    def enu_wp_x_set_callback(self,msg):
        self.enu_wp_x_set_received = True
        temp_set_x = np.array(msg.data)
        self.enu_wp_x_set = temp_set_x.flatten()
    def enu_wp_y_set_callback(self, msg):
        self.enu_wp_y_set_received = True
        temp_set_y = np.array(msg.data)
        self.enu_wp_y_set = temp_set_y.flatten()
    def wp_idx_callback(self,msg):
        self.wp_idx_received = True
        self.wp_idx = msg.data
        if (self.enu_wp_x_set_received == True and self.enu_wp_y_set_received == True):
            print(self.enu_wp_x_set)

            self.enu_wp_x = (self.enu_wp_x_set[self.wp_idx])
            self.enu_wp_y = self.enu_wp_y_set[self.wp_idx]
            

    # def enu_pos_callback(self, msg):
    #     self.enu_pos_received = True
    #     self.enu_pos = [msg.x, msg.y,msg.z]
    #     if(len(self.heading_q)!=0):
    #         # Create a marker message
    #         marker = Marker()
    #         # Set the necessary properties of the marker
    #         marker.header.frame_id = "map"
    #         marker.id = 1
    #         marker.type = Marker.ARROW
    #         marker.action = Marker.ADD
    #         marker.pose.position.x = self.enu_pos[0]
    #         marker.pose.position.y = self.enu_pos[1]
    #         marker.pose.position.z = self.enu_pos[2]
    #         marker.pose.orientation.x = self.heading_q[0]
    #         marker.pose.orientation.y = self.heading_q[1]
    #         marker.pose.orientation.z = self.heading_q[2]
    #         marker.pose.orientation.w = self.heading_q[3]
    #         marker.scale.x = 0.2
    #         marker.scale.y = 0.2
    #         marker.scale.z = 0.2
    #         marker.color.r = 1.0
    #         marker.color.g = 0.0
    #         marker.color.b = 0.0
    #         marker.color.a = 1.0
    #         print("publish my pos")
    #         self.viz_pose_pub.publish(marker)

    #         # Create a marker message
    #         marker2 = Marker()
    #         # Set the necessary properties of the marker2
    #         marker2.header.frame_id = "laser_frame"
    #         marker2.id = 8
    #         marker2.type = Marker.ARROW
    #         marker2.action = Marker.ADD
    #         marker2.pose.position.x = self.enu_pos[0]
    #         marker2.pose.position.y = self.enu_pos[1]
    #         marker2.pose.position.z = self.enu_pos[2]
    #         marker2.pose.orientation.x = self.heading_q[0]
    #         marker2.pose.orientation.y = self.heading_q[1]
    #         marker2.pose.orientation.z = self.heading_q[2]
    #         marker2.pose.orientation.w = self.heading_q[3]
    #         marker2.scale.x = 0.2
    #         marker2.scale.y = 0.2
    #         marker2.scale.z = 0.2
    #         marker2.color.r = 1.0
    #         marker2.color.g = 0.0
    #         marker2.color.b = 0.0
    #         marker2.color.a = 1.0
    #         print("publish my pos in l")
    #         self.viz_pose_l_pub.publish(marker2)

    #         #wp
    #     if(self.enu_wp_x_set_received==True and self.enu_wp_y_set_received == True):
    #         # Create a marker message
    #         marker = Marker()
    #         # Set the necessary properties of the marker
    #         marker.header.frame_id = "map"
    #         marker.id = 2
    #         marker.type = Marker.SPHERE
    #         marker.action = Marker.ADD
    #         marker.pose.position.x = self.enu_wp_x_set[0]
    #         marker.pose.position.y = self.enu_wp_y_set[0]
    #         marker.pose.position.z = 0.0
    #         marker.pose.orientation.x = 0.0
    #         marker.pose.orientation.y = 0.0
    #         marker.pose.orientation.z =0.0
    #         marker.pose.orientation.w = 0.0
    #         marker.scale.x = 0.2
    #         marker.scale.y = 0.2
    #         marker.scale.z = 0.2
    #         marker.color.r = 0.0
    #         marker.color.g = 1.0
    #         marker.color.b = 0.0
    #         marker.color.a = 1.0
    #         print("publish my wp 1pos")
    #         self.viz_wp1_pub.publish(marker)


    #         #wp2
    #         # Create a marker message
    #         marker = Marker()
    #         # Set the necessary properties of the marker
    #         marker.header.frame_id = "map"
    #         marker.id = 3
    #         marker.type = Marker.SPHERE
    #         marker.action = Marker.ADD
    #         marker.pose.position.x = self.enu_wp_x_set[1]
    #         marker.pose.position.y = self.enu_wp_y_set[1]
    #         marker.pose.position.z = 0.0
    #         marker.pose.orientation.x = 0.0
    #         marker.pose.orientation.y = 0.0
    #         marker.pose.orientation.z =0.0
    #         marker.pose.orientation.w = 0.0
    #         marker.scale.x = 0.2
    #         marker.scale.y = 0.2
    #         marker.scale.z = 0.2
    #         marker.color.r = 0.0
    #         marker.color.g = 0.0
    #         marker.color.b = 1.0
    #         marker.color.a = 1.0
    #         print("publish my wp2 pos")
    #         self.viz_wp2_pub.publish(marker)

    #                     #wp3
    #         # Create a marker message
    #         marker = Marker()
    #         # Set the necessary properties of the marker
    #         marker.header.frame_id = "map"
    #         marker.id = 4
    #         marker.type = Marker.SPHERE
    #         marker.action = Marker.ADD
    #         marker.pose.position.x = self.enu_wp_x_set[2]
    #         marker.pose.position.y = self.enu_wp_y_set[2]
    #         marker.pose.position.z = 0.0
    #         marker.pose.orientation.x = 0.0
    #         marker.pose.orientation.y = 0.0
    #         marker.pose.orientation.z =0.0
    #         marker.pose.orientation.w = 0.0
    #         marker.scale.x = 0.2
    #         marker.scale.y = 0.2
    #         marker.scale.z = 0.2
    #         marker.color.r = 0.0
    #         marker.color.g = 1.0
    #         marker.color.b = 1.0
    #         marker.color.a = 1.0
    #         print("publish my wp3 pos")
    #         self.viz_wp3_pub.publish(marker)

    #                     #wp4
    #         # Create a marker message
    #         marker = Marker()
    #         # Set the necessary properties of the marker
    #         marker.header.frame_id = "map"
    #         marker.id = 5
    #         marker.type = Marker.SPHERE
    #         marker.action = Marker.ADD
    #         marker.pose.position.x = self.enu_wp_x_set[3]
    #         marker.pose.position.y = self.enu_wp_y_set[3]
    #         marker.pose.position.z = 0.0
    #         marker.pose.orientation.x = 0.0
    #         marker.pose.orientation.y = 0.0
    #         marker.pose.orientation.z =0.0
    #         marker.pose.orientation.w = 0.0
    #         marker.scale.x = 0.2
    #         marker.scale.y = 0.2
    #         marker.scale.z = 0.2
    #         marker.color.r = 1.0
    #         marker.color.g = 1.0
    #         marker.color.b = 0.0
    #         marker.color.a = 1.0
    #         print("publish my wp3 pos")
    #         self.viz_wp4_pub.publish(marker)

    # def viz_obs_phi_pub(self, msg):
    #     self.des_heading_received = True
    #     self.des_heading = [msg.x, msg.y,msg.z, msg.w]
    #     # viz/des_heading
    #     marker = Marker()
    #     # Set the necessary properties of the marker
    #     marker.header.frame_id = "odom"
    #     marker.id = 5
    #     marker.type = Marker.ARROW
    #     marker.action = Marker.ADD
    #     marker.pose.position.x = self.enu_wp_x_set[3]
    #     marker.pose.position.y = self.enu_wp_y_set[3]
    #     marker.pose.position.z = 0.0
    #     marker.pose.orientation.x = 0.0
    #     marker.pose.orientation.y = 0.0
    #     marker.pose.orientation.z =0.0
    #     marker.pose.orientation.w = 0.0
    #     marker.scale.x = 0.2
    #     marker.scale.y = 0.2
    #     marker.scale.z = 0.2
    #     marker.color.r = 1.0
    #     marker.color.g = 1.0
    #     marker.color.b = 1.0
    #     marker.color.a = 1.0
    #     print("publish my obs ")
    #     self.viz_wp4_pub.publish(marker)



            
                
def main(args=None):
    rclpy.init(args=args)
    visualizer = Visualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()