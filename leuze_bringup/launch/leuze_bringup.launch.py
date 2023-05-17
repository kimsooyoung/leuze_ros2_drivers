import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config_file_path = os.path.join(
        get_package_share_directory('leuze_bringup'), 'config', 'rsl400.yaml'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('leuze_bringup'), 'rviz', 'rsl_view.rviz'
    )

    udp_ip = "192.168.10.1"
    udp_port = "9990"

    leuze_rsl_driver = Node(
        package='leuze_rsl_driver',
        executable='leuze_rsl_driver',
        output='screen',
        arguments=[udp_ip, udp_port],
        parameters = [config_file_path]
    )
    
    # Launch RViz
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        TimerAction(
            period=1.0,
            actions=[rviz2]
        ),

        leuze_rsl_driver,
    ])