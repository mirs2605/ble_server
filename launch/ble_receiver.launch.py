"""Start the BLE GATT receiver and route converted messages to ROS topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('default_frame_id', default_value='map'),
        DeclareLaunchArgument('cleaning_zone_topic', default_value='/cleaning_zone'),
        DeclareLaunchArgument('initial_pose_topic', default_value='/initialpose'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='ble_server',
            executable='ble_receiver_node',
            name='ble_receiver_node',
            output='screen',
            parameters=[{
                'default_frame_id': LaunchConfiguration('default_frame_id'),
                'cleaning_zone_topic': LaunchConfiguration('cleaning_zone_topic'),
                'initial_pose_topic': LaunchConfiguration('initial_pose_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            condition=IfCondition(LaunchConfiguration('enabled')),
        ),
    ])
