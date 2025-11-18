"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/real_robot_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
                    'sigterm_timeout',
                    default_value='30'
                ),
        Node(
            package='sas_robot_client_dummy',
            executable='sas_robot_client_dummy_node',
            name='dummy_1',
            namespace="sas",
            output="screen",
            parameters=[{
                #"watchdog_period_in_seconds": LaunchConfiguration('watchdog_period'),
                #"watchdog_maximum_acceptable_delay":1.0,
                "thread_sampling_time_sec": 0.002,  
                "topic_prefix":"dummy_1"  
            }]
        ),

    ])
