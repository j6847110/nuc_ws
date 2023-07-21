# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point, TwistWithCovarianceStamped
from microstrain_inertial_msgs.msg import FilterHeading
from std_msgs.msg import Bool, Int8, Int32, Float32, Float64, String, Float64MultiArray, Int32MultiArray
import numpy as np


class Obstacle_Avoidance(Node):
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'
    def __init__(self):
        super().__init__("obstacle_avoidance")

        # default_params = {
        #     # to be modified
        #     "Left_Bottom": [37.4557583, 126.9517448],  # to be modified
        #     "Right_Bottom": [37.4558121667, 126.9517401667],
        #     "Left_Top": [37.4556311667, 126.9518098333],  # to be modified
        #     "Right_Top": [37.4556311667, 126.9518098333],  # to be modified
        #     "origin": [37.4557583, 126.9517448],  # to be modified, same as Left_Bottom
        # }
        # self.Left_Bottom = self.declare_parameter(
        #     "Left_Bottom", default_params["Left_Bottom"]
        # ).value
        # self.Right_Bottom = self.declare_parameter(
        #     "Right_Bottom", default_params["Right_Bottom"]
        # ).value
        # self.Left_Top = self.declare_parameter(
        #     "Left_Top", default_params["Left_Top"]
        # ).value
        # self.Left_Top = self.declare_parameter(
        #     "Right_Top", default_params["Right_Top"]
        # ).value

        # self.origin = self.declare_parameter("origin", default_params["origin"]).value
        self.dt = 0.1
        self.cur_wp_idx = 0

        #subscriber

        #subscribe obstacle information
        self.obs_labels_sub = self.create_subscription(
            Int32MultiArray, "/obs/labels", self.obs_labels_callback, 1
        )
        self.obs_r_sub = self.create_subscription(
            Float64MultiArray, "/obs/r", self.obs_r_callback, 1
        )
        self.obs_phi_sub = self.create_subscription(
            Float64MultiArray, "/obs/phi", self.obs_phi_callback, 1
        )
        self.obs_x_sub = self.create_subscription(
            Float64MultiArray, "/obs/x", self.obs_x_callback, 1
        )
        self.obs_y_sub = self.create_subscription(
            Float64MultiArray, "/obs/y", self.obs_y_callback, 1
        )
        
        #subscribe from gps
        self.enu_pos_sub = self.create_subscription(
            Point, "/enu_pos", self.enu_pos_callback, 1
        )

        #subscribe from imu
        self.heading_sub = self.create_subscription(
            FilterHeading, "/nav/heading", self.heading_callback, 1
        )

        #subscribe from gps 
        # self.spd_sub = self.create_subscription(String, "/spd", self.spd_callback, 1)
        self.spd_sub = self.create_subscription(
            TwistWithCovarianceStamped, "/spd", self.spd_callback, 1
        )
        
        self.enu_wp_x_set_sub = self.create_subscription(
            Float64MultiArray, "/enu_wp_set/x", self.enu_wp_x_set_callback, 1
        )
        self.enu_wp_y_set_sub = self.create_subscription(
            Float64MultiArray, "/enu_wp_set/y", self.enu_wp_y_set_callback, 1
        )

        #publisher
        self.des_heading_pub = self.create_publisher(Float64, "/des_heading", 1)
        self.des_spd_pub = self.create_publisher(Float64, "/des_spd", 1)
        self.cur_wp_idx_pub = self.create_publisher(Int8, "/wp_idx", 1)
        self.wp_check_pub = self.create_publisher(Bool,"/wp_check",1)

        self.des_pub = self.create_timer(self.dt, self.pub_des)
        
        self.obs_labels_received = False
        self.obs_r_received = False
        self.obs_phi_received = False
        self.obs_x_received = False
        self.obs_y_received = False

        self.enu_pos_received = False
        self.heading_received = False
        self.spd_received = False
        self.obstacles_received = False
        self.enu_wp_x_received = False
        self.enu_wp_y_received = False

        # why 10?
        self.enu_pos = np.zeros((10, 2))
        self.enu_wp_set = np.zeros((10, 2))
        self.heading = np.zeros(10)
        self.spd = np.zeros(10)

        self.wp_reach_check = False
        self.wp_time_cnt = 0
        self.goal_tol = 1.5
        self.wp_state = False
        self.wp_stay_time = 30

        # 
        self.ref_spd = 1
        self.safe_radius = 1.5
        self.safe_heading = []
        self.heading_cost = []
        self.des_heading = np.zeros(10)  # why 10?
        # self.des_spd = np.zeros(10)
        self.des_spd = np.ones(10)
        self.obs_labels = []
        
        self.obs_r=[]
        self.obs_phi=[]
        self.obs_x=[]
        self.obs_y=[]
        self.enu_wp_x_set=[]
        self.enu_wp_y_set=[]

    def wait_for_topics(self):
        self.check_topic_status()
        # self.togodist()

    def check_topic_status(self):
        if not self.enu_pos_received:
            self.get_logger().info("No topic enu_pos_received")
        if not self.heading_received:
            self.get_logger().info("No topic heading_received")
        if not self.spd_received:
            self.get_logger().info("No topic spd_received")
        if not self.obstacles_received:
            self.get_logger().info("No topic obstacles_received")
        if (
            self.enu_pos_received
            and self.heading_received
            and self.spd_received
            and self.obs_labels_received
            and self.enu_wp_x_received
        ):
            self.get_logger().info("All topics received")
        else:
            self.get_logger().info("Waiting for topics to be published")

    # def togodist(self):
    #     dist = np.linalg.norm([self.enu_pos[-1, 0] - self.enu_wp_x_set[self.cur_wp_idx], self.enu_pos[-1, 1] - self.enu_wp_y_set[self.cur_wp_idx]])
    #     self.get_logger().info('To go distance: ' + str(dist))

    def obs_labels_callback(self, msg):
        self.obs_labels_received = True
        # print(self.obs_labels_received)
        self.obs_labels = np.array(msg.data)
        # self.obs_labels = np.reshape(self.obs_labels, (1, -1))
        # self.obs_labels = self.obs_labels.flatten()

    def obs_r_callback(self, msg):
        self.obs_r_received = True
        self.obs_r = np.array(msg.data)
        # self.obs_r = np.reshape(self.obs_r, (1, -1))
        # self.obs_labels = self.obs_labels.flatten()
        
    def obs_phi_callback(self, msg):
        self.obs_phi_received = True
        self.obs_phi = np.array(msg.data)
        # self.obs_phi = np.reshape(self.obs_phi, (1, -1))
        # self.obs_phi = self.obs_phi.flatten()

    def obs_x_callback(self, msg):
        self.obs_x_received = True
        self.obs_x = np.array(msg.data)
        # self.obs_x = np.reshape(self.obs_x, (1, -1))
        # self.obs_x = self.obs_x.flatten()

    def obs_y_callback(self, msg):
        self.obs_y_received = True
        self.obs_y = np.array(msg.data)
        # self.obs_y = np.reshape(self.obs_y, (1, -1))
        # self.obs_y = self.obs_y.flatten()

    def enu_pos_callback(self, msg):
        self.enu_pos_received = True
        # print(self.enu_pos_received)
        self.enu_pos = np.append(self.enu_pos, [[msg.x, msg.y]], axis=0)
        self.enu_pos = self.enu_pos[1:]

    def enu_wp_x_set_callback(self, msg):
        self.enu_wp_x_received = True
        # print(self.enu_wp_x_received)
        temp_set_x = np.array(msg.data)
        self.enu_wp_x_set = temp_set_x.flatten()
        
    def enu_wp_y_set_callback(self, msg):
        self.enu_wp_y_received = True
        temp_set_y = np.array(msg.data)
        self.enu_wp_y_set = temp_set_y.flatten()
        # self.enu_wp_set = np.append(self.enu_wp_x_set, self.enu_wp_y_set, axis=0)
        # self.enu_wp_set = np.transpose(self.enu_wp_set)
        dist = np.linalg.norm([self.enu_pos[-1, 0] - self.enu_wp_x_set[self.cur_wp_idx], self.enu_pos[-1, 1] - self.enu_wp_y_set[self.cur_wp_idx]])
        self.get_logger().info('To go dist: ' + str(dist))
        self.wp_reach_check = (
            dist < self.goal_tol
        )
        if self.wp_reach_check == True:
            if self.wp_state == False:
                self.get_logger().info("Changing waypoint ...")
                self.cur_wp_idx += 1
                self.wp_state = True
            else:  # self.wp_state = True
                if self.wp_time_cnt < self.wp_stay_time:
                    self.wp_time_cnt += 1
                    self.wp_state = True
                else:  # self.wp_time_cnt > self.wp_stay_time
                    self.wp_state = False
                    self.wp_time_cnt = 0
        else:  # wp_reach_check == False:
            self.wp_state = False

        # Waypoint mission clear check
        if self.cur_wp_idx >= len(self.enu_wp_x_set):
            self.get_logger().info("Waypoint Mission Clear")
            return

    def heading_callback(self, msg):
        self.heading_received = True
        # print(self.heading_received)
        self.heading = np.append(self.heading, msg.heading_rad)
        self.heading = self.heading[1:]

    def spd_callback(self, msg):
        self.spd_received = True
        # print(self.spd_received)
        # self.spd = np.append(self.spd, float(msg.data))
        u = msg.twist.twist.linear.x
        v = msg.twist.twist.linear.y
        vel = np.array([u, v])
        spd = np.linalg.norm(vel)
        self.spd = np.append(self.spd, spd)
        self.spd = self.spd[1:]

    def pub_des(self):
        # print(self.enu_pos_received, self.heading_received, self.spd_received, self.obs_labels_received, self.enu_wp_x_received)
        # print(2222)
        if (
            self.enu_pos_received
            and self.heading_received
            and self.spd_received
            and self.obs_labels_received
            and self.obs_r_received
            and self.obs_phi_received
            and self.obs_x_received
            and self.obs_y_received
            and self.enu_wp_x_received
        ):  # all topic received
            # obs information not matched
            if len(self.obs_labels) == len(self.obs_r) \
                and len(self.obs_labels) == len(self.obs_phi) \
                and len(self.obs_labels) == len(self.obs_x) \
                and len(self.obs_labels) == len(self.obs_y) :
                # print(333)
                self.cal_des()
            else:
                return       
            
        else:   # topic not received yet

            return
            
        print("cur_wp_idx", self.cur_wp_idx, "err_heading", self.des_heading[-1] - self.heading[-1], "des_spd", self.des_spd[-1],"wp_check",self.wp_reach_check)

        des_heading = Float64()
        des_heading.data = self.des_heading[-1]
        self.des_heading_pub.publish(des_heading)
            
        des_spd = Float64()
        des_spd.data = self.des_spd[-1]
        self.des_spd_pub.publish(des_spd)

        cur_wp_idx_ = Int8()
        cur_wp_idx_.data = self.cur_wp_idx
        self.cur_wp_idx_pub.publish(cur_wp_idx_)

        wp_check = Bool()
        wp_check.data = self.wp_reach_check
        self.wp_check_pub.publish(wp_check)

    def cal_des(self):
        if self.wp_reach_check == True and self.wp_time_cnt < self.wp_stay_time:
            # print(1)
            self.des_spd = np.append(self.des_spd, 0)
            self.des_spd = self.des_spd[1:]
            des_heading = np.arctan2(self.enu_wp_y_set[self.cur_wp_idx] - cur_pos[1], self.enu_wp_x_set[self.cur_wp_idx] - cur_pos[0])
            self.des_heading = np.append(self.des_heading, des_heading)
            self.des_heading = self.des_heading[1:]
            self.wp_time_cnt += 1
        elif self.wp_reach_check == True and self.wp_time_cnt >= self.wp_stay_time:
            # print("2")
            self.wp_reach_check = False
            self.cur_wp_idx +=1
            if self.cur_wp_idx > len(self.enu_wp_set[:, 0]):
                self.get_logger().info('Goal Reached')
                return
        else:  # self.wp_state = False:
            # print("3")
            cur_pos = self.enu_pos[-1, :]
            # print(cur_pos)
            # wp ref heading
            # print(self.cur_wp_idx)
            # print(self.enu_wp_y_set)
            # print(self.enu_wp_y_set[self.cur_wp_idx])
            # print(self.enu_wp_y_set[self.cur_wp_idx] - cur_pos[1])
            self.ref_heading = np.arctan2(self.enu_wp_y_set[self.cur_wp_idx] - cur_pos[1], self.enu_wp_x_set[self.cur_wp_idx] - cur_pos[0])
            # print(self.ref_heading)           
            #### calculate des_heading and des_spd
            if len(self.obs_labels) != 0:
                self.danger_heading = []
                self.safe_heading = []
                self.heading_cost = []

                # if len(self.obs_labels) - len(self.obs_r) != 0:
                #     print("ERROR_r")
                # if len(self.obs_labels) - len(self.obs_phi) != 0:
                #     print("ERROR_phi")
                # if len(self.obs_labels) - len(self.obs_x) != 0:
                #     print("ERROR_x")
                # if len(self.obs_labels) - len(self.obs_y) != 0:
                #     print("ERROR_y")
                #     return
                # if len(self.obs_labels) - len(self.obs_r) == 0 \
                #     and len(self.obs_labels) - len(self.obs_x) == 0 \
                #     and len(self.obs_labels) - len(self.obs_y) == 0 \
                #     and len(self.obs_labels) - len(self.obs_phi) == 0 :
                #       print("GODD")

                for idx, label in enumerate(self.obs_labels):
                    #danger obs
                    if self.obs_r[idx] < self.safe_radius:
                        
                        danger_heading = self.obs_phi[idx] + self.heading[-1]
                        if danger_heading > np.pi:
                            danger_heading -= 2*np.pi
                        elif danger_heading < -np.pi:
                            danger_heading += 2*np.pi
                            
                        self.danger_heading = np.append(
                            self.danger_heading, danger_heading
                        )
                        continue
                    #safe obs
                    else:
                        safe_heading = self.obs_phi[idx] + self.heading[-1]
                        if safe_heading > np.pi:
                            safe_heading -= 2*np.pi
                        elif safe_heading < -np.pi:
                            safe_heading += 2*np.pi
                        # print(safe_heading)
                        # print(self.ref_heading)
                        heading_cost = abs(safe_heading - self.ref_heading)
                        if heading_cost > np.pi:
                            heading_cost -= 2*np.pi
                        elif heading_cost < -np.pi:
                            heading_cost += 2*np.pi
                            
                        self.safe_heading = np.append(self.safe_heading, safe_heading)
                        self.heading_cost = np.append(self.heading_cost, heading_cost)
                        
                if len(self.safe_heading) != 0:
                    des_heading_idx = np.argmin(self.heading_cost)
                    des_heading = self.safe_heading[des_heading_idx]

                    self.des_spd = np.append(self.des_spd, self.ref_spd)
                    self.des_spd = self.des_spd[1:]
                    self.des_heading = np.append(self.des_heading, des_heading)
                    self.des_heading = self.des_heading[1:]
                else:
                    return
                
            else:
                self.des_spd = np.append(self.des_spd, self.des_spd[-1])
                self.des_spd = self.des_spd[1:]
                self.des_heading = np.append(self.des_heading, self.des_heading[-1])
                self.des_heading = self.des_heading[1:]


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = Obstacle_Avoidance()
    obstacle_avoidance.wait_for_topics()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
