from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    namespace = '/snuboat'
    navigator_frequency = 10.0
    imu_frequency = 10.0
    gps_frequency = 10.0
    WP_tol = 0.5
    Kp_pwm = 1
    Kd_pwm = 1
    Kp_ang = 1
    Kd_ang = 1
    ref_speed = 1

    return LaunchDescription([
        Node(
            package='snuboat_navigation', 
            executable='wp_following',
            output='screen',
            emulate_tty=True,
            name='WP_Following',
            parameters = [
                {'namespace' : namespace},
            	{'navigator_frequency' : navigator_frequency},
                {'imu_frequency' : imu_frequency},
                {'gps_frequency' : gps_frequency},
                {'WP_tol' : WP_tol},
                {'Kp_pwm' : Kp_pwm},
                {'Kd_pwm' : Kd_pwm},
                {'Kp_ang' : Kp_ang},
                {'Kd_ang' : Kd_ang},
                {'ref_speed' : 1}
                ],
            ),
    ])
