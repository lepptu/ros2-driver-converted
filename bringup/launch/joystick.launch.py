from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    joy_params = PathJoinSubstitution([
        FindPackageShare("hoverboard_driver"),
        "config",
        "joystick.yaml"
    ])

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'publish_stamped_twist': True}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )

    joy_to_arduino_node = Node(
            package='hoverboard_driver',
            executable='joy_to_arduino.py',
            name='joy_to_arduino',
            parameters=[joy_params] # Tämä on kriittinen: lukee arvot joystick.yaml:sta
        )

    return LaunchDescription([
        joy_node,
        teleop_node,
        joy_to_arduino_node
    ])