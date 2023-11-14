## Notes
real_moveit_config: ur_real.urdf.xacro
- No Camera Link

sim_moveit_config: ur_moveit.urdf.xacro
- Camera Link

## Edited Files (From Directories Outside final_project)
rg2_gripper.urdf.xacro
- Base Joint Orientation

ur_moveit.urdf.xacro
- rg2 gripper 
- Removed Ground Box Link

ur_real.urdf.xacro
- Renamed

## Launch Files
world.launch.py
- Duplicate of the_construct_office_gazebo/launch/start_world_standalone.launch.py

arm.launch.py
- Spawns Arm in Gazebo
- Robot State Publisher 
    - use_sim_time: True

controllers_sim.launch.py
- Joint State Broadcaster
- Joint Trajectory Controller 
- Gripper Controller

controllers_real.launch.py
- Robot State Publisher
    - use_sim_time: False
- Joint State Broadcaster
- Joint Trajectory Controller
- Gripper Controller

setup_sim.launch.py
- Move Group Node
    - use_sim_time: True
- RVIZ

setup_real.launch.py
- Move Group Node
    - use_sim_time: False
- RVIZ

## TODO:
- controllers_real.launch.py 
    - Clean Up and Finalize
- Add RVIZ Config Files
- Check Files for Conflicts
- Add Real and Sim MTC Program Nodes
- Edit World File
