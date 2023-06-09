import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import TimerAction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='autonomous_waiter_description').find('autonomous_waiter_description')
    default_model_path = os.path.join(pkg_share, 'description/autonomous_waiter_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config/urdf_config.rviz')

    lifecycle_nodes = ['controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother']
    
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                #   ('/cmd_vel', '/diff_cont/cmd_vel_unstamped'),
                #   ('/diff_cont/odom', '/odom')
                  ]
    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'default_nav_through_poses_bt_xml': LaunchConfiguration('default_nav_through_poses_bt_xml'),
        'default_nav_to_pose_bt_xml': LaunchConfiguration('default_nav_to_pose_bt_xml'),
        'autostart': LaunchConfiguration('autostart'),
        'map_subscribe_transient_local': LaunchConfiguration('map_subscribe_transient_local')
    }

    nav2_configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params_file'),
        root_key=LaunchConfiguration('namespace'),
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # robot_state_publisher_node = launch_ros.actions.Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    # )
    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    # )
    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )
    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', default_rviz_config_path],
    # )

    nav2_controller_server = launch_ros.actions.Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[nav2_configured_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=remappings + [('/odom', '/diff_cont/odom'), ('cmd_vel', 'cmd_vel_nav')]
    )

    nav2_smoother_server = launch_ros.actions.Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[nav2_configured_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=remappings
    )

    nav2_planner_server = launch_ros.actions.Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[nav2_configured_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=remappings
    )

    nav2_behavior_server = launch_ros.actions.Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[nav2_configured_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=remappings + [('cmd_vel', '/diff_cont/cmd_vel_unstamped')]
    )

    nav2_bt_navigator = launch_ros.actions.Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[nav2_configured_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=remappings + [('/odom', '/diff_cont/odom')]
    )

    nav2_waypoint_follower = launch_ros.actions.Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[nav2_configured_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=remappings
    )

    nav2_velocity_smoother = launch_ros.actions.Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[nav2_configured_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=remappings +
                [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/diff_cont/cmd_vel_unstamped')]
    )

    nav2_lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': LaunchConfiguration('autostart')},
                    {'node_names': lifecycle_nodes}]
    )

    # delayed_nav2_nodes = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=robot_state_publisher_node,
    #         on_start=[nav2_controller_server,
    #             nav2_smoother_server,
    #             nav2_planner_server,
    #             nav2_behavior_server,
    #             nav2_bt_navigator,
    #             nav2_waypoint_follower,
    #             nav2_velocity_smoother,
    #             nav2_lifecycle_manager]
    #     )
    # )

    return launch.LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument('autostart', default_value='true',
                                            description='Automatically startup the nav2 stack'),
        launch.actions.DeclareLaunchArgument('default_nav_through_poses_bt_xml',
                                            default_value=os.path.join(
                                                pkg_share,
                                                'config', 'navigate_through_poses_w_replanning_and_recovery.xml'),
                                            description='Full path to the behavior tree xml file to use'),
        launch.actions.DeclareLaunchArgument('default_nav_to_pose_bt_xml',
                                            default_value=os.path.join(
                                                pkg_share,
                                                'config', 'navigate_to_pose_w_replanning_and_recovery.xml'),
                                            description='Full path to the behavior tree xml file to use'),
        launch.actions.DeclareLaunchArgument('nav2_params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
                                            description='Full path to the ROS2 parameters file to use'),
        launch.actions.DeclareLaunchArgument('map_subscribe_transient_local', default_value='true',
                                            description='Whether to set the map subscriber QoS to transient local'),
        launch.actions.DeclareLaunchArgument('namespace', default_value='',
                                            description='Top-level namespace'),
        launch.actions.DeclareLaunchArgument('use_composition', default_value='False',
                                            description='Use composed bringup if True'),
        launch.actions.DeclareLaunchArgument('container_name', default_value='nav2_container',
                                            description='the name of conatiner that nodes will load in if use composition'),
        launch.actions.DeclareLaunchArgument('use_respawn', default_value='False',
                                            description='Whether to respawn if a node crashes. Applied when composition is disabled.'),
        launch.actions.DeclareLaunchArgument('log_level', default_value='info',
                                            description='log level'),
        # robot_state_publisher_node,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        # delayed_nav2_nodes,
        # rviz_node,
        nav2_controller_server,
        nav2_smoother_server,
        nav2_planner_server,
        nav2_behavior_server,
        nav2_bt_navigator,
        nav2_waypoint_follower,
        nav2_velocity_smoother,
        nav2_lifecycle_manager
    ])