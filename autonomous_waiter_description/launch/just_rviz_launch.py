import launch
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='autonomous_waiter_description').find('autonomous_waiter_description')
    default_rviz_config_path = os.path.join(pkg_share, 'config/urdf_config.rviz')

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    return launch.LaunchDescription([
        rviz_node
    ])