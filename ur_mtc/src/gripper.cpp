// ALTERED FROM MAIN TO NOT GRASP CUP TO ALLOW FOR MULTIPLE RUNS BEFORE TESTING WITH GRASP
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <cmath>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_control");

namespace mtc = moveit::task_constructor;
/* Sim Notes
UR Gazebo Pose 13.8, -18.56, 1.032
Cup Gazebo Pose 14.0, -18.2, 1.09
cup_radius 0.035
cup_height 0.09
Coffee Machine Gazebo Pose 14.15, -17.9, 1
Camera Link -0.15 0.35 0.35 / 13.65 -18.11
*/

struct poses{
    float x;
    float y;
    float z;
    float rx;
    float py;
    float yz;      
};

struct objects{
    float length;
    float width;
    float height;
};

struct linear_movement{
    float min;
    float max;
    float x;
    float y;
    float z;
};

// Approachs and Retreats 
// Set Z 1.0 to use min/max range. Otherwise Z Absolute Distance Relative to Gripper 
linear_movement grasp_approach = {0.02, 0.03, 0.0, 0.0, 1.0};
linear_movement grasp_retreat = {0.01, 0.05, 0.0, 0.0, 1.0};
linear_movement fill_approach = {0.01, 0.05, 0.0, 0.0, 1.0};
linear_movement fill_retreat = {0.0, 0.0, -0.05, -0.05, 0.0};
linear_movement place_approach = {0.045, 0.06, 0.0, 0.0, -1.0};
linear_movement place_retreat = {0.05, 0.1, 0.0, 0.0, 1.0};
linear_movement regrasp_approach = {0.01, 0.03, 0.0, 0.0, -1.0};
linear_movement regrasp_retreat = {0.0, 0.0, 0.0, 0.0, 0.05};
linear_movement preserve = {0.0, 0.0, -0.2, -0.2, 0.0};
linear_movement serve_approach = {0.0, 0.0, 0.0, 0.0, -0.15};
linear_movement serve_retreat = {0.0, 0.0, 0.0, 0.0, 0.15};

// Object Dimensions Length, Width, Height
objects counter = {0.5, 0.65, 0.04};
objects subcounter = {0.5, 0.5, 0.04};
objects machine = {0.3, 0.27, 0.33};
objects machine_top = {0.06, 0.27, 0.12};
objects machine_dispenser = {0.08, 0.07, 0.04};
objects handle = {0.1, 0.03, 0.04};
objects clamp = {0.15, 0.03, 0.04};

float cup_height = 0.09;
float cup_radius = 0.035;
// Object Poses
poses cup_pose = {0.21, 0.41, 0.035, 0.0, 0.0, 0.0};
poses counter_pose = {0.16, 0.0, -0.02, 0.0, 0.0, 0.0};
poses subcounter_pose = {0.16, 0.4, -0.0325, 0.0, 0.0, 0.0};
poses machine_pose = {0.38, 0.67, 0.145, 0.0, 0.0, 0.0};
poses machine_top_pose = {0.2, 0.67, 0.25, 0.0, 0.0, 0.0};
poses machine_dispenser_pose = {0.2, 0.67, 0.17, 0.0, 0.0, 0.0};
poses handle_pose = {0.12, 0.51, 0.02, 0.0, 0.0, 0.0};
poses clamp_pose = {-0.06, 0.51, 0.02, 0.0, 0.0, 0.0};

// IK Poses
poses fill_pose = {0.14, 0.64, 0.09, 0.0, 0.0, 0.0}; // Check and Adjust Incrementally
poses regrasp_position = {-0.02, 0.4, 0.085, 0.0, 0.0, 0.0};
poses serve_pose = {-0.3, 0.25, -0.3, 0.0, 0.0, 0.0};
// IK Transforms
poses grasp_frame_transform = {0.0, 0.02, 0.225, M_PI/2, 0.0, 0.0}; // {x, y, z, rx, py, yz} Orients gripper and open/close horizontal
poses regrasp_frame_transform = {0.0, 0.0, 0.235, M_PI, 0.0, 0.0}; 

// Adjustment for Cup X-Axis Rotation (Radians)
double cup_offset = -0.0855;

// Utility Functions
Eigen::Isometry3d toEigen(const poses& val) {
	return Eigen::Translation3d(val.x, val.y, val.z) *
	       Eigen::AngleAxisd(val.rx, Eigen::Vector3d::UnitX()) *
	       Eigen::AngleAxisd(val.py, Eigen::Vector3d::UnitY()) *
	       Eigen::AngleAxisd(val.yz, Eigen::Vector3d::UnitZ());
}
geometry_msgs::msg::Pose toPose(const poses& val) {
	return tf2::toMsg(toEigen(val));
}

class UrControl
{
public:
    UrControl(const rclcpp::NodeOptions & options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
    bool doTask();
    void setupPlanningScene();

private:
    mtc::Task createTask();   
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;  
}; // END UR CONTROL CLASS

moveit_msgs::msg::CollisionObject createCup(const poses &cup_pose, const float &cup_height, const float &cup_radius)
{
    geometry_msgs::msg::Pose pose = toPose(cup_pose);
	moveit_msgs::msg::CollisionObject object;
	object.id = "cup";
	object.header.frame_id = "world";
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = { cup_height, cup_radius };
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::msg::CollisionObject createBox(std::string name, const poses &box_pose, const objects &box)
{
    geometry_msgs::msg::Pose pose = toPose(box_pose);
	moveit_msgs::msg::CollisionObject object;
	object.id = name;
	object.header.frame_id = "world";
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
	object.primitives[0].dimensions = { box.length, box.width, box.height }; // Relative to World X Y Z  
	object.primitive_poses.push_back(pose);
	return object;
}

void spawnObjects(moveit::planning_interface::PlanningSceneInterface &psi, const moveit_msgs::msg::CollisionObject &object)
{
    psi.applyCollisionObject(object);
}

// Definitions
UrControl::UrControl(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("control_node", options) }
{

}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr UrControl::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void UrControl::setupPlanningScene()
{
    rclcpp::sleep_for(std::chrono::microseconds(200)); // Wait for ApplyPlanningScene Service
    moveit::planning_interface::PlanningSceneInterface psi;
    //spawnObjects(psi, createCup(cup_pose, cup_height, cup_radius));
    spawnObjects(psi, createBox("counter", counter_pose, counter));
    spawnObjects(psi, createBox("subcounter", subcounter_pose, subcounter));
    spawnObjects(psi, createBox("machine", machine_pose, machine));
    spawnObjects(psi, createBox("machine top", machine_top_pose, machine_top));
    spawnObjects(psi, createBox("machine dispenser", machine_dispenser_pose, machine_dispenser));
    spawnObjects(psi, createBox("handle", handle_pose, handle));
    spawnObjects(psi, createBox("clamp", clamp_pose, clamp));
    spawnObjects(psi, createCup(cup_pose, cup_height, cup_radius));
}

bool UrControl::doTask()
{

    task_ = createTask();
    
    try
    {
        task_.init();
    }
    catch (mtc::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return false;
    }

    if (task_.plan(3))
    {
        
        RCLCPP_INFO(LOGGER, "Task planning succeded");
        
    } else {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return false;
    }
    task_.introspection().publishSolution(*task_.solutions().front());
    
    //auto result = task_.execute(*task_.solutions().front());

    return true;
}

mtc::Task UrControl::createTask()
{
    mtc::Task task;

    task.stages()->setName("gripper test");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "ur_manipulator";
    const auto& gripper_group_name = "gripper";
    const auto& gripper_frame = "tool0"; 

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", gripper_group_name); 
    task.setProperty("hand", gripper_group_name);
	task.setProperty("hand_grasping_frame", gripper_frame);
    task.setProperty("ik_frame", gripper_frame); 

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    interpolation_planner->setProperty("goal_joint_tolerance", 10.0);

    // ***STAGES***

    // ADD TO READY POSITION?
    // ***Close Gripper <MoveTo>***
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
        stage->setGroup(gripper_group_name);
        stage->setGoal("close");       
        stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
        task.add(std::move(stage));    
    }

    // ***Open Gripper <MoveTo>***
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
        stage->setGroup(gripper_group_name);
        stage->setGoal("open"); 
        stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
        // -> Set Current State Pointer ->
        //current_state_ = stage.get();
        task.add(std::move(stage));
    }

    // ***Close Gripper <MoveTo>***
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
        stage->setGroup(gripper_group_name);
        stage->setGoal("close");       
        stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
        task.add(std::move(stage));    
    }

    // ***Open Gripper <MoveTo>***
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
        stage->setGroup(gripper_group_name);
        stage->setGoal("open"); 
        stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
        // -> Set Current State Pointer ->
        //current_state_ = stage.get();
        task.add(std::move(stage));
    }

    return task;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_node = std::make_shared<UrControl>(options);
    //auto perception_node = std::make_shared<Perception>();
    
    rclcpp::executors::MultiThreadedExecutor exec;
    /*
    exec.add_node(perception_node);
    // Wait for Perception
    while(!perception_node->cylinder_found)
    {
        exec.spin_once();
        std::cout << "Waiting on Perception" << std::endl;
        rclcpp::sleep_for(std::chrono::milliseconds(15000));
    }
    //perception_node->addCup();
    exec.remove_node(perception_node);
    */
    auto spin_thread = std::make_unique<std::thread>([&exec, &mtc_node]() {
        std::cout << "Thread Start" << std::endl;
        //exec.add_node(perception_node);
        exec.add_node(mtc_node->getNodeBaseInterface());
        std::cout << "Pre-Spin" << std::endl;
        exec.spin();
        exec.remove_node(mtc_node->getNodeBaseInterface());
        //exec.remove_node(perception_node);
    });
    //mtc_node->setupPlanningScene();

    // Wait for Octomap Update
    //std::cout << "Waiting for Octomap Update" << std::endl;
    //rclcpp::sleep_for(std::chrono::milliseconds(2000));   
    mtc_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();

    return 0;
} // END MAIN

