import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

from launch_ros.actions import SetRemap, Node

def generate_launch_description():
    # Etsitään oman pakettisi (hoverboard_driver) kansio
    pkg_dir = get_package_share_directory('hoverboard_driver')
    
    # Määritellään polku juuri luomaasi nav2_params.yaml -tiedostoon
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Etsitään Nav2:n oma käynnistyskansio
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[nav2_params_path, {'autostart': True}],
        output='screen'
    )
    
    # Otetaan käyttöön Nav2:n pelkkä navigointiosuus (ei AMCL-paikannusta, koska GPS hoitaa sen)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params_path,
            'use_composition': 'True',
            'container_name': 'nav2_container',  # Varmistetaan, että se löytää säiliön
        }.items()
        
    )

    return LaunchDescription([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
        nav2_container,
        navigation_launch
    ])