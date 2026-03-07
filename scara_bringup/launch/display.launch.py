import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
)


def generate_launch_description():
    scara_urdf_path = os.path.join(get_package_share_directory('scara_description'), 'urdf', 'scara.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_directory('scara_bringup'), 'config', 'rviz_config.rviz')
    
    ld = LaunchDescription()
    
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": Command(["xacro ", scara_urdf_path])}
            ],
        )
    )
    
    ld.add_action(
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
    )
    
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=["-d", rviz_config_path],
        )
    )
    
    return ld