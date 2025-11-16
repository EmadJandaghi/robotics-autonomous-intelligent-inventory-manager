
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='autonomous_inventory_manager', executable='go_to_aoi', name='go_to_aoi', output='screen'),
        Node(package='autonomous_inventory_manager', executable='object_detection', name='object_detection', output='screen'),
        Node(package='autonomous_inventory_manager', executable='inventory_checking', name='inventory_checking', output='screen'),
    ])
