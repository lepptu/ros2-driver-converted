import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():
    pkg_share = FindPackageShare('hoverboard_driver')

    config = PathJoinSubstitution([pkg_share, 'config', 'fusioncore.yaml'])

    fusioncore_node = Node(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[config],
        remappings=[
            ("/imu/data", "/imu/data"),
            ("/odom/wheels", "/hoverboard_base_controller/odom"),
            ("/gnss/fix", "/ublox_dgnss/fix"),
        ]
    )


    return LaunchDescription([
            fusioncore_node,
        ])
