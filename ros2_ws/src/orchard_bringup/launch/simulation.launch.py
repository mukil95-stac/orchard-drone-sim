from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package='orchard_bringup',executable='orchard_sim',name='orchard_sim',output='screen')])
