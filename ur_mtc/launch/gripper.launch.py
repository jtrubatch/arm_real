from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_mtc',
            executable='gripper_action',
            output='screen'),
    ])