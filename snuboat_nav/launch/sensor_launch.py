#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    # create and return launch description object
    return LaunchDescription(
        [
        
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': 'dev/ttyLIDAR', 
                         'serial_baudrate': '256000', 
                         'frame_id': 'laser',
                         'inverted': 'false', 
                         'angle_compensate': 'true', 
                         'scan_mode': 'Sensitivity'}],
            output='screen'),
            
            # #IMU
            ExecuteProcess(
                cmd=["ros2", "launch", "microstrain_inertial_driver", "microstrain_launch.py"], output="screen"
            ),

            
        ]
    )
