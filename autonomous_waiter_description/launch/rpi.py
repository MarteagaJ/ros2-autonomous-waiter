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
    # default_slam_params_file = os.path.join(pkg_share, 'config/mapper_params_online_async.yaml')
    default_slam_params_file = os.path.join(pkg_share, 'config/mapper_params_online_sync.yaml')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    controller_params_file = os.path.join(get_package_share_directory("autonomous_waiter_description"),'config','my_controllers.yaml')

    controller_manager = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        # arguments=['--ros-args', '--log-level', "debug"],
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
                    controller_params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
        # arguments=["diff_cont", '--ros-args', '--log-level', "debug"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
        # arguments=["joint_broad", '--ros-args', '--log-level', "debug"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    rplidar_node = launch_ros.actions.Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
            'frame_id': 'laser_link',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # slam_node = launch_ros.actions.Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     output='screen',
    #     parameters=[
    #       default_slam_params_file,
    #       {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ]
    # )

    slam_node = launch_ros.actions.Node(
        parameters=[
          default_slam_params_file,
          {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    delayed_slam_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rplidar_node,
            on_start=[slam_node],
        )
    )

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
        launch.actions.DeclareLaunchArgument('default_bt_xml_filename',
                                            default_value=os.path.join(
                                                pkg_share,
                                                'config', 'navigate_through_poses_w_replanning_and_recovery.xml'),
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
        robot_state_publisher_node,
        joint_state_publisher_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        rplidar_node,
        delayed_slam_node
    ])