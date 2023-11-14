from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess

from launch.event_handlers import OnProcessExit

def generate_launch_description():

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'

    )
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    joint_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_controller],
        )

    )

    gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_trajectory_controller,
            on_exit=[load_gripper_controller]
        )
    )

    return LaunchDescription([
        load_joint_state_controller,
        joint_controller_spawner,
        gripper_controller_spawner,
    ])
