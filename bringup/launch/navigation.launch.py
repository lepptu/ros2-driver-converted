import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Etsitään oman pakettisi (hoverboard_driver) kansio
    pkg_dir = get_package_share_directory('hoverboard_driver')
    
    # Määritellään polku juuri luomaasi nav2_params.yaml -tiedostoon
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Lista Nav2:n käynnistettävistä lifecycle-solmuista
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'collision_monitor'
    ]

    # Yhteiset parametrit kaikille Nav2-solmuille
    node_parameters = [nav2_params_path, {'use_sim_time': False}]

    # Alkuperäiset Nav2-remappaukset tf/tf_static -topicceihin + oma cmd_vel -remappaus
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
        #('/cmd_vel', 'cmd_vel_raw')  # Ohjaimet julkaisevat nyt raakaa nopeutta
    ]

    # Säiliö (container), jonka sisällä kaikki Nav2-solmut jaettavat muistinsa (Composition)
    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[nav2_params_path, {'autostart': True}],
        output='screen',
        #arguments=['--ros-args', '--log-level', 'debug']
        #arguments=['--ros-args', '--log-level', 'debug']
    )

    # Ladataan kaikki yllä luetellut komponentit kerralla käyntiin yllä olevaan säiliöön
    load_composable_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            # 1. Controller Server
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=node_parameters,
                remappings=remappings + [('cmd_vel', 'cmd_vel_raw')]
            ),
            
            # 2. Smoother Server
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=node_parameters,
                remappings=remappings
            ),
            
            # 3. Planner Server
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=node_parameters,
                remappings=remappings
            ),
            
            # 4. Behavior Server
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=node_parameters,
                remappings=remappings + [('cmd_vel', 'cmd_vel_raw')]
            ),
            
            # 5. BT Navigator
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=node_parameters,
                remappings=remappings
            ),
            
            # 6. Waypoint Follower
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=node_parameters,
                remappings=remappings
            ),
            
            # 7. Velocity Smoother
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=node_parameters,
                remappings=remappings + [('cmd_vel', 'cmd_vel_raw'), ('cmd_vel_smoothed', 'cmd_vel_nav')]
            ),

            # 8. Collision Monitor
            ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=node_parameters,
                remappings=remappings
            ),
            
            # 9. Lifecycle Manager
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[
                    {'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}
                ]
            )
        ]
    )

    # N4 (BT_REVIEW): publishes the KeepoutFilter mask from mow-area holes +
    # web-drawn keepout zones. Lives with the costmaps so the filter always
    # has its inputs, mission running or not.
    keepout_mask_node = Node(
        package='mowing_navigation',
        executable='keepout_mask_publisher',
        name='keepout_mask_publisher',
        output='screen',
    )

    return LaunchDescription([
        nav2_container,
        load_composable_nodes,
        keepout_mask_node,
    ])