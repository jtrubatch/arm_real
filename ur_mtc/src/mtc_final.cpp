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
#include "ur_mtc/perception.hpp"
#include <iostream>
#include <cmath>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_control");
namespace mtc = moveit::task_constructor;

// Variables 
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

// Scene Object Dimensions Length, Width, Height
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
poses cup_pose = {0.10, 0.41, 0.035, 0.0, 0.0, 0.0};
poses counter_pose = {0.16, 0.0, -0.02, 0.0, 0.0, 0.0};
poses subcounter_pose = {0.16, 0.4, -0.0325, 0.0, 0.0, 0.0};
poses machine_pose = {0.38, 0.67, 0.145, 0.0, 0.0, 0.0};
poses machine_top_pose = {0.2, 0.67, 0.25, 0.0, 0.0, 0.0};
poses machine_dispenser_pose = {0.2, 0.67, 0.17, 0.0, 0.0, 0.0};
poses handle_pose = {0.12, 0.51, 0.02, 0.0, 0.0, 0.0};
poses clamp_pose = {-0.06, 0.51, 0.02, 0.0, 0.0, 0.0};

// IK Target Poses
poses fill_pose = {0.14, 0.64, 0.09, 0.0, 0.0, 0.0}; 
poses regrasp_position = {-0.02, 0.42, 0.085, 0.0, 0.0, 0.0};
poses serve_pose = {-0.29, 0.21, -0.48, 0.0, 0.0, 0.0}; 
poses prehome_pose = {-0.3, 0.2, 0.15, 0.0, 0.0, 0.0};

// IK Transforms Offsets and Orientation {x, y, z, rx, py, yz} 
poses grasp_frame_transform = {0.0, 0.02, 0.20, M_PI/2, 0.0, 0.0}; 
poses regrasp_frame_transform = {0.0, 0.0, 0.24, M_PI, 0.0, 0.0}; 
poses prehome_frame_transform = {0.0, 0.0, 0.0, M_PI, 0.0, 0.0}; 

// Approachs and Retreats 
// Set Z 1.0 to use min/max range. Otherwise Z Absolute Distance Relative to Gripper 
linear_movement grasp_approach = {0.075, 0.085, 0.0, 0.0, 1.0};
linear_movement grasp_retreat = {0.01, 0.05, 0.0, 0.0, 1.0};
linear_movement fill_approach = {0.01, 0.05, 0.0, 0.0, 1.0};
linear_movement fill_retreat = {0.0, 0.0, -0.05, -0.05, 0.0};
linear_movement place_approach = {0.045, 0.06, 0.0, 0.0, -1.0};
linear_movement place_retreat = {0.1, 0.15, 0.0, 0.0, 1.0};
linear_movement regrasp_approach = {0.05, 0.075, 0.0, 0.0, -1.0};
linear_movement regrasp_retreat = {0.0, 0.0, 0.0, 0.0, 0.05};
linear_movement preserve = {0.0, 0.0, -0.2, -0.2, 0.0};
linear_movement serve_approach = {0.0, 0.0, 0.0, 0.0, -0.15};
linear_movement serve_retreat = {0.0, 0.0, 0.0, 0.0, 0.15};

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

// Scene Object Functions
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

void setupPlanningScene()
{
    rclcpp::sleep_for(std::chrono::microseconds(200)); // Wait for ApplyPlanningScene Service
    moveit::planning_interface::PlanningSceneInterface psi;
    spawnObjects(psi, createBox("counter", counter_pose, counter));
    spawnObjects(psi, createBox("subcounter", subcounter_pose, subcounter));
    spawnObjects(psi, createBox("machine", machine_pose, machine));
    spawnObjects(psi, createBox("machine top", machine_top_pose, machine_top));
    spawnObjects(psi, createBox("machine dispenser", machine_dispenser_pose, machine_dispenser));
    spawnObjects(psi, createBox("handle", handle_pose, handle));
    spawnObjects(psi, createBox("clamp", clamp_pose, clamp));
    spawnObjects(psi, createCup(cup_pose, cup_height, cup_radius));
}

// Ur Control Class
class UrControl
{
public:
    UrControl(const rclcpp::NodeOptions& options) : node_{ std::make_shared<rclcpp::Node>("control_node", options)}
    {
    }
    
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }
    
    bool doTask()
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
        // Uncomment for Auto-Execution
        //auto result = task_.execute(*task_.solutions().front());
        return true;
    }
private:
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;  

    mtc::Task createTask()
    {
        mtc::Task task;

        task.stages()->setName("Coffee PnP");
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
  
        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        sampling_planner->setProperty("goal_joint_tolerance", 1e-4);
        sampling_planner->setProperty("planning_attempts", 2);
        
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(0.001);
        cartesian_planner->setJumpThreshold(1.75);
        
        // ***STAGES***
        // -> Current State Pointer ->
        mtc::Stage* current_state_ = nullptr;
        {
            auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");

            // Verify that object is not attached
            auto applicability_filter =
                std::make_unique<mtc::stages::PredicateFilter>("applicability test", std::move(current_state));
            applicability_filter->setPredicate([object = "cup"](const mtc::SolutionBase& s, std::string& comment) {
                if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
                    comment = "object is already attached and cannot be picked";
                    return false;
                }
                return true;
            });
            task.add(std::move(applicability_filter));
        }
        // ADD TO READY POSITION?
        // ***Close Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("close"); 

            task.add(std::move(stage));    
        }

        // ***Open Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("open"); 
            // -> Set Current State Pointer ->
            current_state_ = stage.get();
            task.add(std::move(stage));
        }

        // ***Move to Pre-Grasp Position <Connector>***
        {
            auto stage = std::make_unique<mtc::stages::Connect>(
            "pre-grasp position", mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(stage));
        }
        // -> Grasp Stage Pointer ->
        mtc::Stage* grasp_stage_ = nullptr;
        
        // ***Initial Grasp Container***
        {
            auto grasp = std::make_unique<mtc::SerialContainer>("initial grasp");
            // Pass Properties from Task to Container
            task.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
            grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});
            
            // ***Approach Cup <MoveRelative>***
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
                stage->properties().set("marker_ns", "approach cup");
                stage->properties().set("link", gripper_frame);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setIKFrame(gripper_frame);
                stage->setMinMaxDistance(grasp_approach.min, grasp_approach.max);

                // Set hand forward direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = gripper_frame;
                vec.vector.z = grasp_approach.z;
                stage->setDirection(vec);
                grasp->insert(std::move(stage));
            }

            // ***Grasp Pose <Generator>***
            {
                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate initial grasp pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "initial_grasp_pose"); 
                stage->setPreGraspPose("open");
                stage->setObject("cup");
                stage->setAngleDelta(M_PI / 32); 
                stage->setMonitoredStage(current_state_);
            
                // ***Compute IK <Wrapper>***
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("initial grasp pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(20);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame(toEigen(grasp_frame_transform), gripper_frame); 
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                grasp->insert(std::move(wrapper));
            }

            // ***Allow Collision <PlanningScene>*** 
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
                stage->allowCollisions(
                    "cup", 
                    task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
                    true);
                grasp->insert(std::move(stage));
            }    
            // ***Close Gripper <MoveTo>***
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
                stage->setGroup(gripper_group_name);
                stage->setGoal("close");      
                grasp->insert(std::move(stage));    
            }
            // ***Attach Cup <PlanningScene>***  
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cup");
                stage->attachObject("cup", gripper_frame);
                grasp->insert(std::move(stage));
            }
            // *** Lift Cup <MoveRelative>***
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cup", cartesian_planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(grasp_retreat.min, grasp_retreat.max); 
                stage->setIKFrame(gripper_frame);
                stage->properties().set("marker_ns", "lift_cup");

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.z = grasp_retreat.z;
                stage->setDirection(vec);
        
                grasp->insert(std::move(stage));
            }
            
            // -> Set Grasp Stage Pointer ->
            grasp_stage_ = grasp.get();
            task.add(std::move(grasp));
        } // END INITIAL GRASP CONTAINER

        // *** Move to Coffee Machine <Connector>***
        {
            auto stage = std::make_unique<mtc::stages::Connect>("move to coffee machine", 
                mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
                                                        //{gripper_group_name, sampling_planner}
            stage->setTimeout(15.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(stage));
        }   
        // ***Fill Coffee Container***
        // -> Fill Stage Pointer ->
        mtc::Stage* fill_stage_ = nullptr;
        {
            auto fill = std::make_unique<mtc::SerialContainer>("fill coffee");
            task.properties().exposeTo(fill->properties(), {"eef", "hand", "group", "ik_frame"});
            fill->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});
            // *** Move to Coffee Machine <MoveRelative>***
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("fill cup", cartesian_planner);
                stage->properties().set("marker_ns", "fill_cup");
                stage->properties().set("link", gripper_frame);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(fill_approach.min, fill_approach.max);

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = gripper_frame;
                vec.vector.z = fill_approach.z; 
                stage->setDirection(vec);
                fill->insert(std::move(stage));
            }
            // *** Fill Coffee Pose <Generator>***
            {
                auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate fill pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "fill_pose");
                stage->setObject("cup");
                // Define Pose 
                geometry_msgs::msg::PoseStamped fill_pose_msg;
                fill_pose_msg.header.frame_id = "world";
                fill_pose_msg.pose = toPose(fill_pose);
                stage->setPose(fill_pose_msg);
                stage->setMonitoredStage(grasp_stage_);

                // Compute IK
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("fill pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(20);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame("cup");
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                fill->insert(std::move(wrapper));
            }

            // ***Retract from Coffee Machine <MoveRelative>***
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("retract coffee1", cartesian_planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setIKFrame(gripper_frame);
                stage->properties().set("marker_ns", "retract_coffee1");
                // Set Direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.x = fill_retreat.x;
                vec.vector.y = fill_retreat.y;
                stage->setDirection(vec);
                fill->insert(std::move(stage));
            }
            fill_stage_ = fill.get();
            task.add(std::move(fill));
        } // END FILL STAGE

        // *** Move to Regrasp <Connector> ***
        {
            auto stage = std::make_unique<mtc::stages::Connect>("move to regrasp position", 
                mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
                                                        //{gripper_group_name, sampling_planner}
            stage->setTimeout(15.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(stage));
        }         

        // *** Move to Regrasp Container ***
        // -> Regrasp Stage Pointer ->
        mtc::Stage* regrasp_position_stage_ = nullptr;
        {
            auto regrasp = std::make_unique<mtc::SerialContainer>("regrasp position");
            task.properties().exposeTo(regrasp->properties(), {"eef", "hand", "group", "ik_frame"});
            regrasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

            // *** Regrasp Position <Generator>***
            {
                auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate regrasp position");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "regrasp_position");
                stage->setObject("cup");
                // Define Pose 
                geometry_msgs::msg::PoseStamped regrasp_position_msg;
                regrasp_position_msg.header.frame_id = "world";
                regrasp_position_msg.pose = toPose(regrasp_position);
                stage->setPose(regrasp_position_msg);
                stage->setMonitoredStage(fill_stage_);

                // Compute IK
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("regrasp position IK", std::move(stage));
                wrapper->setMaxIKSolutions(20);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame("cup");
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                regrasp->insert(std::move(wrapper));
            }

            // ***Lower Cup <MoveRelative>***
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("lower cup", cartesian_planner);
                stage->properties().set("marker_ns", "lower_cup");
                stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                stage->setMinMaxDistance(place_approach.min, place_approach.max);

                // Set down direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.z = place_approach.z; 
                stage->setDirection(vec);
                regrasp->insert(std::move(stage));
            }

            // ***Open Gripper <MoveTo>***
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
                stage->setGroup(gripper_group_name);
                stage->setGoal("open");
                regrasp->insert(std::move(stage));
            }

            // ***Disable Collision <PlanningScene>***
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("disable collision");
                stage->allowCollisions("cup", 
                    task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
                    false); 
                regrasp->insert(std::move(stage));
            }

            // ***Detach Cup <PlanningScene>***
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cup");
                stage->detachObject("cup", gripper_frame);
                regrasp->insert(std::move(stage));
            }

            // ***Retreat from Cup <MoveRelative>***
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat from cup", cartesian_planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(place_retreat.min, place_retreat.max);
                stage->setIKFrame(gripper_frame);
                stage->properties().set("marker_ns", "retreat");

                //Set Retreat Direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.z = place_retreat.z;
                stage->setDirection(vec);
                regrasp->insert(std::move(stage));
            }
            // -> Set Regrasp Position Stage Pointer ->
            regrasp_position_stage_ = regrasp.get();
            task.add(std::move(regrasp));
        } // END REGRASP POSITION CONTAINER

        // ***Move to Regrasp <Connector>***
        {
            auto stage = std::make_unique<mtc::stages::Connect>(
            "move to regrasp", mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(stage));
        }
        
        // -> Regrasp Stage Pointer ->
        mtc::Stage* regrasp_stage_ = nullptr;
        // ***Regrasp Container***
        {
            auto regrasp = std::make_unique<mtc::SerialContainer>("regrasp cup");
            task.properties().exposeTo(regrasp->properties(), {"eef", "hand", "group", "ik_frame"});
            regrasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

            // ***Approach Cup <MoveRelative>***
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("approach regrasp", cartesian_planner);
                stage->properties().set("marker_ns", "approach regrasp");
                stage->properties().set("link", gripper_frame);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(regrasp_approach.min, regrasp_approach.max);

                // Set Direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.z = regrasp_approach.z; 
                stage->setDirection(vec);
                regrasp->insert(std::move(stage));
            }

            // ***Generate Grasp Pose <Generator>***
            {
                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate regrasp pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "regrasp_pose");
                stage->setPreGraspPose("open");
                stage->setObject("cup");
                stage->setAngleDelta(M_PI / 16);
                stage->setMonitoredStage(regrasp_position_stage_);

                // ***Compute IK***
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("regrasp pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(20);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame(toEigen(regrasp_frame_transform), gripper_frame);
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
                regrasp->insert(std::move(wrapper)); 
            }

            // *** Allow Collision <PlanningScene>***
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
                stage->allowCollisions(
                    "cup",
                    task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
                    true);
                regrasp->insert(std::move(stage));
            }
        
            // ***Close Gripper <MoveTo>***
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
                stage->setGroup(gripper_group_name);
                stage->setGoal("close");
                regrasp->insert(std::move(stage));    
            }

            // ***Attach Cup <PlanningScene>***
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cup");
                stage->attachObject("cup", gripper_frame);
                regrasp->insert(std::move(stage));
            }

            // ***Lift Cup <MoveRelative>***
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cup", cartesian_planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setIKFrame(gripper_frame);
                stage->properties().set("marker_ns", "lift_cup");

                // Set direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.z = regrasp_retreat.z;
                stage->setDirection(vec);
                regrasp->insert(std::move(stage));
            }

            // ***Pre-Serve <MoveRelative>***
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("pre-serve", cartesian_planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setIKFrame(gripper_frame);
                stage->properties().set("marker_ns", "pre-serve");

                // Set direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.x = preserve.x;
                vec.vector.y = preserve.y;
                stage->setDirection(vec);
                regrasp->insert(std::move(stage));
            }
            // -> Set Regrasp Stage Pointer ->
            regrasp_stage_ = regrasp.get(); 
            task.add(std::move(regrasp));
        } // END REGRASP CONTAINER
        // *** Serve <Connector> ***
        {
            auto stage = std::make_unique<mtc::stages::Connect>(
            "move to serve", mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);

            task.add(std::move(stage));
        }
        // *** Serve Container ***
        mtc::Stage* serve_stage_ = nullptr;
        {
            auto serve = std::make_unique<mtc::SerialContainer>("serve coffee");
            task.properties().exposeTo(serve->properties(), {"eef", "hand", "group", "ik_frame"});
            serve->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});
            
            // ***Serve to Baristabot <MoveRelative>***
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("move to baristabot", cartesian_planner);
                stage->properties().set("marker_ns", "serve_cup");
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
               
                // Set direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.z = serve_approach.z; 
                stage->setDirection(vec);
                serve->insert(std::move(stage));
            }   
            // *** Serve Coffee Pose <Generator>***
            {
                auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate serve pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "serve_pose");
                stage->setObject("cup");
                // Define Pose 
                geometry_msgs::msg::PoseStamped serve_pose_msg;
                serve_pose_msg.header.frame_id = "world";
                serve_pose_msg.pose = toPose(serve_pose);
                stage->setPose(serve_pose_msg);
                stage->setMonitoredStage(regrasp_stage_);

                // Compute IK
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("serve pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(20);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame("cup");
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                serve->insert(std::move(wrapper));
            }

            //  ***Open Gripper <MoveTo>***
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
                stage->setGroup(gripper_group_name);
                stage->setGoal("open");      
                serve->insert(std::move(stage));    
            }

            // ***Disable Collision <PlanningScene>***
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("disable collision");
                stage->allowCollisions("cup", 
                    task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
                    false); 
                serve->insert(std::move(stage));
            }
            
            // ***Detach Cup <PlanningScene>***
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cup");
                stage->detachObject("cup", gripper_frame);
                serve->insert(std::move(stage));
            }

            // ***Retract from Serve Pose
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("move to baristabot", cartesian_planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setIKFrame(gripper_frame);
                stage->properties().set("marker_ns", "serve_cup");

                // Set direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.z = serve_retreat.z; 
                stage->setDirection(vec);
                serve->insert(std::move(stage));
            }
            serve_stage_ = serve.get();   
            task.add(std::move(serve));
        } // END SERVE CONTAINER

        // ***Move to Pre-Home <Connector>***
        {
            auto stage = std::make_unique<mtc::stages::Connect>(
            "move to prehome", mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(stage));
        }
        // ***Pre-Home <MoveRelative>*** 
        {
            auto stage = std::make_unique<mtc::stages::GeneratePose>("generate prehome pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "serve_pose");
            // Define Pose 
            geometry_msgs::msg::PoseStamped prehome_pose_msg;
            prehome_pose_msg.header.frame_id = "world";
            prehome_pose_msg.pose = toPose(prehome_pose);
            stage->setPose(prehome_pose_msg);
            stage->setMonitoredStage(serve_stage_);

            // Compute IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("prehome pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(20);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(toEigen(prehome_frame_transform), gripper_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            task.add(std::move(wrapper));
        }

        // RETURN HOME
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setGoal("home");
            stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
            task.add(std::move(stage));
        }
        
        return task;
    } // END CREATE TASK
}; // END UR CONTROL CLASS

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_node = std::make_shared<UrControl>(options);
    auto perception_node = std::make_shared<Perception>();
    
    rclcpp::executors::MultiThreadedExecutor exec;

    exec.add_node(perception_node);
    // Wait for Perception
    while(!perception_node->cylinder_found)
    {
        exec.spin_once();
        std::cout << "Waiting on Perception" << std::endl;
        rclcpp::sleep_for(std::chrono::milliseconds(15000));
    }
    perception_node->addCup();
    // Perception Node Removed to Reduce Overhead
    exec.remove_node(perception_node);

    auto spin_thread = std::make_unique<std::thread>([&exec, &mtc_node]() {
        std::cout << "Thread Start" << std::endl;
        exec.add_node(mtc_node->getNodeBaseInterface());
        std::cout << "Pre-Spin" << std::endl;
        exec.spin();
        exec.remove_node(mtc_node->getNodeBaseInterface());
    });
    setupPlanningScene();

    // Wait for Octomap Update
    std::cout << "Waiting for Octomap Update" << std::endl;
    rclcpp::sleep_for(std::chrono::milliseconds(2000));   
    mtc_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();

    return 0;
} // END MAIN