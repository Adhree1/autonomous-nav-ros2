#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz for visualization'
    )
    
    # TurtleBot3 Gazebo world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )
    
    # Autonomous Navigator node
    navigator_node = Node(
        package='autonomous_navigator',
        executable='navigator',
        name='autonomous_navigator',
        output='screen',
        emulate_tty=True
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_rviz_cmd)
    
    # Add nodes
    ld.add_action(gazebo_launch)
    ld.add_action(navigator_node)
    
    return ld
