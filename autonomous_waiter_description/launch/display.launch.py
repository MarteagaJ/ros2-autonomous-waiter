import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='autonomous_waiter_description').find('autonomous_waiter_description')
    default_model_path = os.path.join(pkg_share, 'description/autonomous_waiter_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config/urdf_config.rviz')
    default_slam_params_file = os.path.join(pkg_share, 'config/mapper_params_online_async.yaml')

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
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    controller_params_file = os.path.join(get_package_share_directory("autonomous_waiter_description"),'config','my_controllers.yaml')

    # cmd_vel_publisher_node = launch_ros.actions.Node(
    #     package='cmd_vel_publisher',
    #     executable='cmd_vel_publisher',
    #     parameters=[{'forward_vel': "0"}]
    # )

    controller_manager = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
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
        arguments=["joint_broad"],
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
            'scan_mode': 'Standard'
        }]
    )

    # slam_node = launch_ros.actions.Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     output='screen',
    #     parameters=[
    #       default_slam_params_file,
    #       {'use_sim_time': False}
    #     ]
    # )

    # delayed_slam_node = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=rplidar_node,
    #         on_start=[slam_node],
    #     )
    # )
    # diff_drive_spawner = launch_ros.actions.Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["diff_cont"],
    # )
    # joint_broad_spawner = launch_ros.actions.Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_broad"],
    # )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        # launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                     description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        # controller_manager,
        # cmd_vel_publisher_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        rplidar_node,
        # slam_node
        # delayed_slam_node
    ])