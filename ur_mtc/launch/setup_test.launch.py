from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Configs
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="r85_moveit_config")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .robot_description(file_path="config/name.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )
    
    # RVIZ Config
    rviz_config_file = (
        get_package_share_directory("ur_mtc") + "/config/moveit.rviz"
    )
    
    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}
    planning_scene_monitor_params = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }
    # Octomap
    octomap_config = {'octomap_frame': 'world', 
                      'octomap_resolution': 0.01,
                      'max_range': 5.0}
    
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            planning_scene_monitor_params,
            octomap_config,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.sensors_3d,
        ],
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=move_group_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[rviz_node],
                )
            ]
        )
    )

    return LaunchDescription([
        move_group_node,
        delay_rviz,       
    ])

'''
    # Pipeline Parameters for Constrained Planning
    moveit_config.planning_pipelines["ompl"]["ur_manipulator"][
        "enforce_constrained_state_space"
    ] = True
    
    moveit_config.planning_pipelines["ompl"]["ur_manipulator"][
        "projection_evaluation"
    ] = "joints(should_pan_joint,shoulder_lift_joint,elbow_joint)"
    
    moveit_config.planning_pipelines["ompl"]["manipulator_gripper"][
        "enforce_constrained_state_space"
    ] = True
    
    moveit_config.planning_pipelines["ompl"]["manipulator_gripper"][
        "projection_evaluation"
    ] = "joints(should_pan_joint,shoulder_lift_joint,elbow_joint)"

    # Constraints Path to Generated File for Move Group Node
    #constraints = {"move_group/constraint_approximations_path": "$(find ur_mtc)/config"}
'''