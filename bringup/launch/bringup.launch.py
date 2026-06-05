from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('hoverboard_driver')

    def launch(filename):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', filename])
            )
        )

    return LaunchDescription([
        launch('arduino.launch.py'),
        launch('diffbot.launch.py'),
        launch('gps.launch.py'),
        launch('camera.launch.py'),
        launch('navigation.launch.py'),
    ])
