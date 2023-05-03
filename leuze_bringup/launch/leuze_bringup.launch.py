import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    leuze_rsl_driver = Node(
        package='leuze_rsl_driver',
        executable='leuze_rsl_driver',
        output='screen',
    )

    # # Launch RViz
    # rviz2 = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", rviz_config_file],
    # )

    return LaunchDescription([
        # TimerAction(
        #     period=3.0,
        #     actions=[rviz2]
        # ),

        leuze_rsl_driver,
    ])