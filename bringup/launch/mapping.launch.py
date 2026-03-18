import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Määritetään polku SINUN kopioimaasi parametritiedostoon
    my_pkg_dir = get_package_share_directory('hoverboard_driver')
    default_params_file = os.path.join(my_pkg_dir, 'config', 'mapper_params_online_async.yaml')
    params_file = LaunchConfiguration('params_file', default=default_params_file)

    # Haetaan slam_toolboxin oletus launch-tiedosto
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    # Luodaan IncludeLaunchDescription slam_toolboxin käynnistämiseksi
    # ja välitetään sille 'slam_params_file' -argumentti
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file
        }.items()
    )

    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to SLAM parameter file'
    ))
    
    ld.add_action(start_slam_toolbox_cmd)

    return ld