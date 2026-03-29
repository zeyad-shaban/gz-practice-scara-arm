#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    debug_visuals = LaunchConfiguration('debug_visuals', default='false')

    autofocus_node = Node(
        package='litho_brain',
        executable='autofocus_node',
        output='screen',
        parameters=[{'debug_autofocus': debug_visuals}]
    )
    
    autoalignment_node = Node(
        package='litho_brain',
        executable='autoalignment_node',
        output='screen',
        parameters=[{'debug_autoalignment': debug_visuals}]
    )

    brain_node = Node(
        package='litho_brain',
        executable='litho_brain',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('debug_visuals', default_value='false',
                              description='Show autofocus and Autoalignment debug window'),
        autofocus_node,
        autoalignment_node,
        brain_node,
    ])