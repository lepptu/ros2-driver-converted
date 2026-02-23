import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Haetaan polku konfiguraatiotiedostoon
    # Varmista, että paketin nimi täsmää (tässä hoverboard_driver)
    config = os.path.join(
        get_package_share_directory('hoverboard_driver'),
        'config',
        'arduino_params.yaml'
    )

    # Määritellään ajettava solmu
    arduino_node = Node(
        package='hoverboard_driver',
        executable='arduino_bridge.py',
        name='arduino_bridge',
        output='screen',
        parameters=[config]
    )

    # Palautetaan LaunchDescription, joka sisältää solmun
    return LaunchDescription([
        arduino_node
    ])