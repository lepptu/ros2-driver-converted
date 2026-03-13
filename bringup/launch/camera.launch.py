from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            output='screen',
            parameters=[{
                'camera': '/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a',
                'format': 'XRGB8888',
                'frame_id': 'camera_link_optical',
                'width': 640,
                'height': 360,                
            }]
        )
    ])