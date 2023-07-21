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
            TwistWithCovarianceStamped, "/fix_velocity", self.spd_callback, 1
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
        # self.des_pub = self.create_timer(1.0, self.pub_des)
        # self.des_pub = self.create_timer(3.0, self.pub_des)
        
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
        self.goal_tol = 3.0
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
        #margin
        self.inflate_obs_phi = np.deg2rad(20)

    def wait_for_topics(self):
        self.timer = self.create_timer(10.0, self.check_topic_status)
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
        # print("this is test")
        # print("cur_wp_idx", self.cur_wp_idx, \
        #       "To go:", self.print_dist, \
        #     "des_heading", np.rad2deg(self.des_heading[-1]), \
        #       "des_spd", self.des_spd[-1],\
        #       "wp_check",self.wp_reach_check)

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
        self.print_dist = np.linalg.norm([self.enu_pos[-1, 0] - self.enu_wp_x_set[self.cur_wp_idx], self.enu_pos[-1, 1] - self.enu_wp_y_set[self.cur_wp_idx]])
        print("this is distance to wp")
        print(self.print_dist)
        self.wp_reach_check = bool(
            self.print_dist < self.goal_tol
        )
        print("this is my pos")
        print(self.enu_pos[-1,:])
        print("this is my waypoint")
        print(self.enu_wp_x_set[self.cur_wp_idx],self.enu_wp_y_set[self.cur_wp_idx])
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
    #rad
    def heading_callback(self, msg):
        self.heading_received = True
        # print(self.heading_received)
        self.heading = np.append(self.heading, msg.heading_rad)
        self.heading = self.heading[1:]

    def spd_callback(self, msg):
        self.spd_received = True
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
            # print("all topic received now, obs info check")
            # print("obs - label")
            # print(len(self.obs_labels))
            # print("obs x")
            # print(len(self.obs_x))
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
            
        err_heading = self.ref_heading - self.heading[-1]
        if err_heading > np.pi:
            err_heading -= 2*np.pi
        elif err_heading < -np.pi:
            err_heading += 2*np.pi

        print("cur_wp_idx", self.cur_wp_idx, \
              "des_heading", np.rad2deg(self.des_heading[-1]), \
              "heading", np.rad2deg(self.heading[-1]), \
              "err_heading", np.ceil(np.rad2deg(err_heading)), \
              "des_spd", self.des_spd[-1],\
              "wp_check", self.wp_reach_check)

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
        cur_pos = self.enu_pos[-1, :]
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
            

            self.ref_heading = np.arctan2(self.enu_wp_y_set[self.cur_wp_idx] - cur_pos[1], self.enu_wp_x_set[self.cur_wp_idx] - cur_pos[0])
            print("this is ref heading")
            print(np.rad2deg(self.ref_heading))
            #### calculate des_heading and des_spd
            if len(self.obs_labels) != 0: # if there are scanned obstacles
                self.danger_r = []
                self.danger_phi = []
                self.danger_x = []
                self.danger_y = []
                self.safe_phi = np.linspace(-np.pi, np.pi, 360).transpose()
                # print(np.shape(self.safe_phi))

                self.safe_heading = []

                # safe_phi
                # idx_array = np.where(np.diff(self.obs_labels) != 0)[0] + 1
                idx_array = np.where(np.diff(self.obs_labels) != 0)[0]
                print(len(idx_array))
                # print(idx_array)
                for i, idx in enumerate(idx_array):

                    if idx != idx_array[-1]:
                        # prisnt(len(self.obs_r[idx:idx_array[i+1]-1]))
                        if len(self.obs_r[idx:idx_array[i+1]-1]) > 10:
                            # print(self.obs_r[idx:idx_array[i+1]-1])
                            # print("this is minimum value")
                            # print(np.min(self.obs_r[idx:idx_array[i+1]-1]) )
                            if np.min(self.obs_r[idx:idx_array[i+1]-1]) < self.safe_radius:
                            
                                if self.obs_phi[idx] - self.inflate_obs_phi > -np.pi:
                                    start_phi = self.obs_phi[idx] - self.inflate_obs_phi
                                else:
                                    start_phi = -np.pi

                                if self.obs_phi[idx_array[i+1]-1] + self.inflate_obs_phi < np.pi:
                                    end_phi = self.obs_phi[idx_array[i+1]-1] + self.inflate_obs_phi
                                else:
                                    end_phi = np.pi

                                # print(start_phi, end_phi)
                                temp_safe_phi1 = self.safe_phi
                                safe_idx1 = np.array(temp_safe_phi1 < start_phi)
                                temp_safe_phi1 = temp_safe_phi1[safe_idx1]
                                temp_safe_phi1 = np.reshape(temp_safe_phi1, (1, -1))
                                # print(temp_safe_phi1)
                                temp_safe_phi2 = self.safe_phi
                                safe_idx2 = np.array(temp_safe_phi2 > end_phi)
                                temp_safe_phi2= temp_safe_phi2[safe_idx2]
                                temp_safe_phi2 = np.reshape(temp_safe_phi2, (1, -1))
                                # print(temp_safe_phi2)
                                self.safe_phi = np.append(temp_safe_phi1, temp_safe_phi2)
                                # print(self.safe_phi)
                        
                    else:
                        if len(self.obs_r[idx:]) > 10:
                            if np.min(self.obs_r[idx:]) < self.safe_radius:

                                if self.obs_phi[idx] - self.inflate_obs_phi > -np.pi:
                                    start_phi = self.obs_phi[idx] - self.inflate_obs_phi
                                else:
                                    start_phi = -np.pi

                                if self.obs_phi[-1] + self.inflate_obs_phi < np.pi:
                                    end_phi = self.obs_phi[-1] + self.inflate_obs_phi
                                else:
                                    end_phi = np.pi

                                # print(start_phi, end_phi)
                                temp_safe_phi1 = self.safe_phi
                                safe_idx1 = np.array(temp_safe_phi1 < start_phi)
                                temp_safe_phi1 = temp_safe_phi1[safe_idx1]
                                temp_safe_phi1 = np.reshape(temp_safe_phi1, (1, -1))
                                # print(temp_safe_phi1)
                                temp_safe_phi2 = self.safe_phi
                                safe_idx2 = np.array(temp_safe_phi2 > end_phi)
                                temp_safe_phi2= temp_safe_phi2[safe_idx2]
                                temp_safe_phi2 = np.reshape(temp_safe_phi2, (1, -1))
                                # print(temp_safe_phi2)
                                self.safe_phi = np.append(temp_safe_phi1, temp_safe_phi2)
                                # print(self.safe_phi)

                # print(len(self.safe_phi))   

                self.safe_heading = self.safe_phi + self.heading[-1]
                minus_idx = self.safe_heading > np.pi
                self.safe_heading[minus_idx] -= 2*np.pi
                plus_idx = self.safe_heading < -np.pi
                self.safe_heading[plus_idx] += 2*np.pi
                # print(np.rad2deg(self.safe_heading))

                future_cost = self.safe_heading - self.ref_heading
                future_minus_idx = future_cost > np.pi
                future_cost[future_minus_idx] -= 2*np.pi
                future_plus_idx = future_cost < -np.pi
                future_cost[future_plus_idx] += 2*np.pi

                past_cost = self.safe_heading - self.des_heading[-1]
                past_minus_idx = past_cost > np.pi
                past_cost[past_minus_idx] -= 2*np.pi
                past_plus_idx = past_cost < -np.pi
                past_cost[past_plus_idx] += 2*np.pi

                self.heading_cost = abs(future_cost) + 0.2 * abs(past_cost)
                # print(self.heading_cost)
                # print(np.rad2deg(self.heading_cost))

                # calculate des_heading
                self.des_spd = np.append(self.des_spd, self.ref_spd)
                self.des_spd = self.des_spd[1:]

                if len(self.safe_heading) != 0:
                    des_heading_idx = np.argmin(self.heading_cost)
                    des_heading = self.safe_heading[des_heading_idx]
                    
                    self.des_heading = np.append(self.des_heading, des_heading)
                    self.des_heading = self.des_heading[1:]
                else:
                    return

            else: # if there are no scanned obstacles
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
