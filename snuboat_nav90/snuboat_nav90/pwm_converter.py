# X-CORPS
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point, TwistWithCovarianceStamped
from std_msgs.msg import String, Float64, Int32, Bool, Int64, Float64, Float64MultiArray
from microstrain_inertial_msgs.msg import FilterHeading
import pymap3d as pm
import numpy as np

class PWMConverter(Node):

    def __init__(self):
        super().__init__('pwm_converter')

        default_params = {
            'rudder_lim' : [-50, 50],
            # rudder pwm 500~2500 270 deg
            'rudder_pwm_lim' : [int(1500-50*(1000/135)), int(1500+50*(1000/135))], # 1000: left, 2000: right
            'thrust_pwm_lim' : [1100, 1900], # 1500: stop
            'dt' : 0.1, 
            'Kp_thrust' : 150.0,
            'Kd_thrust' : 50.0,
            'Kp_rudder' : 1.0,
            'Kd_rudder' : 0.1,
            'Kp_self_rot' : 1.0,
            'Kd_self_rot' : 1.0,
            'Kp_DP_x' : 15.0, 
            'Kd_DP_x' : 5.0, 
            'Kp_DP_y' : 15.0, 
            'Kd_DP_y' : 5.0,
            'Kp_DP_hdg' : 15.0,
            'Kd_DP_hdg' : 5.0,
            'lx' : 0.35,
            'ly' : 0.07,
        }
        #params setting
        self.rudder_lim = self.declare_parameter("rudder_lim", default_params['rudder_lim']).value
        self.rudder_pwm_lim = self.declare_parameter("rudder_pwm_lim", default_params['rudder_pwm_lim']).value
        self.thrust_pwm_lim = self.declare_parameter("thrust_pwm_lim", default_params['thrust_pwm_lim']).value
        self.dt = self.declare_parameter("dt", default_params['dt']).value
        self.Kp_thrust = self.declare_parameter("Kp_thrust", default_params['Kp_thrust']).value
        self.Kd_thrust = self.declare_parameter("Kd_thrust", default_params['Kd_thrust']).value
        self.Kp_rudder = self.declare_parameter("Kp_rudder", default_params['Kp_rudder']).value
        self.Kd_rudder = self.declare_parameter("Kd_rudder", default_params['Kd_rudder']).value
        self.Kp_self_rot = self.declare_parameter("Kp_self_rot", default_params['Kp_self_rot']).value
        self.Kd_self_rot = self.declare_parameter("Kd_self_rot", default_params['Kd_self_rot']).value
        self.Kp_DP_x = self.declare_parameter("Kp_DP_x", default_params['Kp_DP_x']).value
        self.Kd_DP_x = self.declare_parameter("Kd_DP_x", default_params['Kd_DP_x']).value
        self.Kp_DP_y = self.declare_parameter("Kp_DP_y", default_params['Kp_DP_y']).value
        self.Kd_DP_y = self.declare_parameter("Kd_DP_y", default_params['Kd_DP_y']).value
        self.Kp_DP_hdg = self.declare_parameter("Kp_DP_hdg", default_params['Kp_DP_hdg']).value
        self.Kd_DP_hdg = self.declare_parameter("Kd_DP_hdg", default_params['Kd_DP_hdg']).value
        self.lx = self.declare_parameter("lx", default_params['lx']).value
        self.ly = self.declare_parameter("ly", default_params['ly']).value
        
        #initialize          
        self.des_heading = np.zeros(10) # [rad]
        self.des_spd = np.zeros(10)
        self.heading = np.zeros(10) # [rad]
        self.spd = np.zeros(10)
        
        self.err_heading = np.zeros(10)
        self.err_spd = np.zeros(10)
        self.err_heading_next = np.zeros(10)
        self.thrust_pwm_L = np.zeros(10)
        self.thrust_pwm_R = np.zeros(10)
        self.rudder_pwm_L = np.zeros(10)
        self.rudder_pwm_R = np.zeros(10)

        self.err_x = np.zeros(10)
        self.err_y = np.zeros(10)
        
        self.wp_check_sub = self.create_subscription(Bool, '/wp_check', self.wp_check_callback, 1)

        self.err_heading_sub = self.create_subscription(Float64, '/err_heading', self.err_heading_callback, 1)
        self.err_heading_next_sub = self.create_subscription(Float64, '/err_heading_next', self.err_heading_next_callback, 1)
        self.err_x_sub = self.create_subscription(Float64, '/err_x', self.err_x_callback, 1)
        self.err_y_sub = self.create_subscription(Float64, '/err_y', self.err_y_callback, 1)
        # self.OS_des_spd_sub =self.create_subscription(Float64, '/des_spd', self.OS_des_spd_callback, 1)
        # self.OS_heading_sub = self.create_subscription(FilterHeading, '/nav/heading', self.OS_heading_callback, 1)
    #    self.spd_sub = self.create_subscription(TwistWithCovarianceStamped, '/fix_velocity', #self.spd_callback, 1)    

        self.pwm_pub =self.create_publisher(Int32, '/pwm', 1)
        
        # self.pwm_timer = self.create_timer(self.dt, self.pub_pwm)
        self.pwm_timer = self.create_timer(0.1, self.pub_pwm)

        self.wp_check = False

        self.des_heading_received = False
        self.des_spd_received = False
        self.heading_received = False
        self.spd_received = False
        self.wp_check_received = False

        self.err_heading_received = False
        self.err_x_received = False
        self.err_y_received = False

        self.err_heading_next_received = False

        self.wp_idx = 0
        self.wp_set_x = []
        self.wp_set_y = []

    def wait_for_topics(self):
        self.timer = self.create_timer(10.0, self.check_topic_status)

    def check_topic_status(self):
        if not self.des_heading_received:
            self.get_logger().info('No topic des_heading_received')
        if not self.des_spd_received:
            self.get_logger().info('No topic des_spd_received')
        if not self.heading_received:
            self.get_logger().info('No topic heading_received')
        if not self.spd_received:
            self.get_logger().info('No topic spd_received')
        if self.des_heading_received and self.des_spd_received and self.heading_received and self.spd_received:
            self.get_logger().info('All topics received')
        else:
            self.get_logger().info('Waiting for topics to be published...')

    #deg transform
    def err_heading_callback(self, msg):
        self.err_heading_received = True
        self.err_heading = np.append(self.err_heading, np.rad2deg(msg.data))
        self.err_heading = self.err_heading[1:]

    def err_heading_next_callback(self, msg):
        self.err_heading_next_received = True
        self.err_heading_next = np.append(self.err_heading_next, np.rad2deg(msg.data))
        self.err_heading_next = self.err_heading_next[1:]

    def err_x_callback(self, msg):
        self.err_x_received = True
        self.err_x = np.append(self.err_x, msg.data)
        self.err_x = self.err_x[1:]

    def err_y_callback(self, msg):
        self.err_y_received = True
        self.err_y = np.append(self.err_y, msg.data)
        self.err_y = self.err_y[1:]

    def wp_check_callback(self, msg):
        self.wp_check_received = True
        self.wp_check = msg.data

    def pub_pwm(self):
        if self.wp_check == False:
            self.rev_idx = 1
            self.cal_thrust_pwm()
            # self.cal_rudder_pwm()
        else:
            # self.cal_self_rotate()
            self.allocate_thrust()

        # thrust_pwm_str_L = str(int(self.thrust_pwm_L[-1])).zfill(4)
        # thrust_pwm_str_R = str(int(self.thrust_pwm_R[-1])).zfill(4)
        # servo_pwm_str_L = str(int(self.rudder_pwm_L[-1])).zfill(4)
        # servo_pwm_str_R = str(int(self.rudder_pwm_R[-1])).zfill(4)
        print("this is thrust pwm str")
        print(self.thrust_pwm_L[-1],self.thrust_pwm_R[-1],self.rudder_pwm_L[-1],self.rudder_pwm_R[-1])
        thrust_pwm_str_L = str(int((self.thrust_pwm_L[-1]-1000)/10)).zfill(2)
        thrust_pwm_str_R = str(int((self.thrust_pwm_R[-1]-1000)/10)).zfill(2)
        servo_pwm_str_L = str(int((self.rudder_pwm_L[-1]-1000)/10)).zfill(2)
        servo_pwm_str_R = str(int((self.rudder_pwm_R[-1]-1000)/10)).zfill(2)
        #Led control + rev direction
        pwm_value = int(thrust_pwm_str_L + thrust_pwm_str_R + servo_pwm_str_L + servo_pwm_str_R)
        print(pwm_value)

        # pwm_value = int(thrust_pwm_str + servo_pwm_str)
        pwm = Int32()
        pwm.data = pwm_value
        print("err heading: ", self.err_heading)
        print("pwm: ", pwm_value)
        self.pwm_pub.publish(pwm)
        
    # functions that return something
    def cal_thrust_pwm(self):
        #PD 
        thrust_pwm_L = 1580 + int((self.err_heading[-1] * self.Kp_self_rot)) # TODO: define the sign!
        thrust_pwm_R = 1580 - int((self.err_heading[-1] * self.Kp_self_rot)) # TODO: define the sign!
        self.thrust_pwm_L = np.append(self.thrust_pwm_L, thrust_pwm_L)
        self.thrust_pwm_L = self.thrust_pwm_L[1:]
        self.thrust_pwm_R = np.append(self.thrust_pwm_R, thrust_pwm_R)
        self.thrust_pwm_R = self.thrust_pwm_R[1:]

        self.rudder_pwm_L = np.append(self.rudder_pwm_L, 1500)
        self.rudder_pwm_L = self.rudder_pwm_L[1:]
        self.rudder_pwm_R = np.append(self.rudder_pwm_R, 1500)
        self.rudder_pwm_R = self.rudder_pwm_R[1:]
    
    def cal_self_rotate(self):
        thrust_pwm_L = 1500 + int((self.err_heading[-1] * self.Kp_self_rot)) # TODO: define the sign!
        thrust_pwm_R = 1500 - int((self.err_heading[-1] * self.Kp_self_rot)) # TODO: define the sign!
        self.thrust_pwm_L = np.append(self.thrust_pwm_L, thrust_pwm_L)
        self.thrust_pwm_L = self.thrust_pwm_L[1:]
        self.thrust_pwm_R = np.append(self.thrust_pwm_R, thrust_pwm_R)
        self.thrust_pwm_R = self.thrust_pwm_R[1:]

        self.rudder_pwm_L = np.append(self.rudder_pwm_L, 1500)
        self.rudder_pwm_L = self.rudder_pwm_L[1:]
        self.rudder_pwm_R = np.append(self.rudder_pwm_R, 1500)
        self.rudder_pwm_R = self.rudder_pwm_R[1:]

    def allocate_thrust(self):
        # Calculate Required Forces 
        Xreq = self.err_x[-1] * self.Kp_DP_x + (self.err_x[-1] - self.err_x[-2]) * self.Kd_DP_x
        Yreq = self.err_y[-1] * self.Kp_DP_y + (self.err_y[-1] - self.err_y[-2]) * self.Kd_DP_y
        Nreq = self.err_heading_next[-1] * self.Kp_DP_hdg + (self.err_heading_next[-1] - self.err_heading_next[-2]) * self.Kd_DP_hdg

        # Allocate thrust
        Config_Mat = np.array([[2, 0, 0, 0, 1, 0, -self.ly], \
                               [0, 2, 0, 0, 1, 0, self.ly], \
                               [0, 0, 2, 0, 0, 1, -self.lx], \
                               [0, 0, 0, 2, 0, 1, -self.lx], \
                               [1, 1, 0, 0, 0, 0, 0], \
                               [0, 0, 1, 1, 0, 0, 0], \
                               [-self.ly, self.ly, -self.lx, -self.lx, 0, 0, 0]])

        Req_Force_Vec = np.array([0, 0, 0, 0, Xreq, Yreq, Nreq]).transpose()
        temp = np.dot(np.linalg.inv(Config_Mat), Req_Force_Vec)
        Thrust_L = np.linalg.norm([temp[0], temp[1]]) * np.sign(temp[0])
        Thrust_R = np.linalg.norm([temp[2], temp[3]]) * np.sign(temp[1])
        Servo_L = np.rad2deg(np.arctan2(temp[1], temp[0])) 
        Servo_R = np.rad2deg(np.arctan2(temp[3], temp[2]))
        # Saturation
        if Servo_L < self.rudder_lim[0] : 
            Servo_L = self.rudder_lim[0]
        elif Servo_L > self.rudder_lim[1]:
            Servo_L = self.rudder_lim[1]
        
        if Servo_R < self.rudder_lim[0]:
            Servo_R = self.rudder_lim[0]
        elif Servo_R > self.rudder_lim[1]:
            Servo_R = self.rudder_lim[1]

        # Map pwm 
        thrust_pwm_L = 1500 + int(Thrust_L) # TODO: define the sign!
        thrust_pwm_R = 1500 + int(Thrust_R) # TODO: define the sign!

        if thrust_pwm_L > self.thrust_pwm_lim[1]:
            thrust_pwm_L = self.thrust_pwm_lim[1]
        elif thrust_pwm_L < self.thrust_pwm_lim[0]:
            thrust_pwm_L = self.thrust_pwm_lim[0]
        
        if thrust_pwm_R > self.thrust_pwm_lim[1]:
            thrust_pwm_R = self.thrust_pwm_lim[1]
        elif thrust_pwm_R < self.thrust_pwm_lim[0]:
            thrust_pwm_R = self.thrust_pwm_lim[0]

        self.thrust_pwm_L = np.append(self.thrust_pwm_L, thrust_pwm_L)
        self.thrust_pwm_L = self.thrust_pwm_L[1:]
        self.thrust_pwm_R = np.append(self.thrust_pwm_R, thrust_pwm_R)
        self.thrust_pwm_R = self.thrust_pwm_R[1:]

        rudder_pwm_L = 1500 -int(Servo_L/135*(1000))
        rudder_pwm_R = 1500 - int(Servo_R/135*(1000))
        self.rudder_pwm_L = np.append(self.rudder_pwm_L, rudder_pwm_L)
        self.rudder_pwm_L = self.rudder_pwm_L[1:]
        self.rudder_pwm_R = np.append(self.rudder_pwm_R, rudder_pwm_R)
        self.rudder_pwm_R = self.rudder_pwm_R[1:]

def main(args=None):
    rclpy.init(args=args)
    pwm_converter = PWMConverter()
    # pwm_converter.wait_for_topics()
    rclpy.spin(pwm_converter)
    pwm_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()