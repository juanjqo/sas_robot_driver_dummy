"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/simulation_example_py_launch.py

"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='sas_robot_driver',
            executable='sas_robot_watchdog_commander_node',
            namespace="sas",
            output="screen",
            parameters=[{
                "robot_name":"dummy_1",
                "thread_sampling_time_sec": 0.01,    
            }]
        ),

    ])
