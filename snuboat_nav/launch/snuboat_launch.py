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
            package='snuboat_nav', 
            executable='lidar',
            output='screen'),
            
            
            
        Node(
            package='snuboat_nav', 
            executable='gnss',
            output='screen'),

        Node(
            package='snuboat_nav', 
            executable='obs',
            output='screen'),
            
            
        Node(
            package='snuboat_nav', 
            executable='pwm_cvt',
            output='screen'),
                        
            

            
        ]
    )
