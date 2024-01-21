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
static const rclcpp::Logger LOGGER2 = rclcpp::get_logger("perception");
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
linear_movement preserve1 = {0.0, 0.0, -0.19, -0.08, 0.0};
linear_movement preserve2 = {0.0, 0.0, -0.08, -0.17, 0.0};
linear_movement serve_approach = {0.0, 0.0, 0.0, 0.0, -0.05};
linear_movement serve_retreat = {0.0, 0.0, 0.0, 0.0, 0.05};

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
float cup2_radius = 0.01;
// Object Poses
poses cup_pose = {0.16, 0.4, 0.035, 0.0, 0.0, 0.0};
poses cup2_pose = {-0.02, 0.4, 0.035, 0.0, 0.0, 0.0};
poses counter_pose = {0.16, 0.0, -0.02, 0.0, 0.0, 0.0};
poses subcounter_pose = {0.16, 0.4, -0.0325, 0.0, 0.0, 0.0};
poses machine_pose = {0.38, 0.67, 0.145, 0.0, 0.0, 0.0};
poses machine_top_pose = {0.2, 0.67, 0.25, 0.0, 0.0, 0.0};
poses machine_dispenser_pose = {0.2, 0.67, 0.17, 0.0, 0.0, 0.0};
poses handle_pose = {0.12, 0.51, 0.02, 0.0, 0.0, 0.0};
poses clamp_pose = {-0.06, 0.51, 0.02, 0.0, 0.0, 0.0};

// IK Poses
poses fill_pose = {0.14, 0.64, 0.12, 0.0, 0.0, 0.0}; // z changed from 0.09 to 0.12 to keep cup in scene but clear it
poses regrasp_position = {-0.02, 0.4, 0.085, 0.0, 0.0, 0.0};
poses serve_pose = {-0.22, 0.05, -0.3, 0.0, 0.0, 0.0};
// IK Transforms
poses grasp_frame_transform = {0.0, 0.02, 0.225, M_PI/2, 0.0, 0.0};
poses grasp_frame_transform2 = {0.0, 0.06, 0.18, M_PI/2, 0.0, 0.0}; // {x, y, z, rx, py, yz} Orients gripper and open/close horizontal
poses regrasp_frame_transform = {0.0, 0.0, 0.18, M_PI, 0.0, 0.0}; 

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

using std::placeholders::_1;

class Perception : public rclcpp::Node
{
public:
    Perception() : Node("perception")
    {
        this->p_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    pc_topic, 1, std::bind(&Perception::cloudCallback, this, _1));
        RCLCPP_INFO(LOGGER2,"Node Created");
    }
    
    void addCup()
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::CollisionObject cup;
        geometry_msgs::msg::Pose pose;
        //Eigen::Vector3d cup_z(cylinder.orientation[0], cylinder.orientation[1], cylinder.orientation[2]);
        //Eigen::Vector3d world_z(0.0, 0.0, 1.0);
        //Eigen::Vector3d axis;
        //axis = world_z.cross(cup_z);
        //axis.normalize();
        //double angle = acos(cup_z.dot(world_z));
        // Using Known Camera Frame Orientation
        
        cup.id = "cup";
        cup.header.frame_id = "camera_depth_optical_frame";
        cup.primitives.resize(1);
        cup.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	    cup.primitives[0].dimensions = { cylinder.height, cylinder.radius };
        pose.orientation.x = 0.716585; //(axis.x() * sin(angle / 2)) + cup_offset);
        pose.orientation.y = 0.0; //axis.y() * sin(angle / 2);
        pose.orientation.z = 0.0; //axis.z() * sin(angle / 2);
        pose.orientation.w = 0.597159; //cos(angle /2);
        //RCLCPP_INFO(LOGGER2, "Orientation X: %f, Y: %f, Z: %f, W: %f,  Angle: %f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, angle);
        pose.position.x = cylinder.center[0];
        pose.position.y = cylinder.center[1]; //+ (0.03 * std::cos(cup_offset));
        pose.position.z = cylinder.center[2]; //+ (0.03 * std::sin(cup_offset));
        cup.primitive_poses.push_back(pose);
        RCLCPP_INFO(LOGGER2, "Cup Radius %.3f, Cup Height %.3f, Cup X %.3f Cup Y %.3f Cup Z %.3f", 
                    cylinder.radius, cylinder.height, cylinder.center[0], cylinder.center[1], cylinder.center[2]);
        psi.applyCollisionObject(cup);
        RCLCPP_INFO(LOGGER2,"Cup Created");
        cup_added = true;
    }

    struct CylinderParams{
        double radius;
        double orientation[3];
        double center[3];
        double height;
        double cLine[3];
    };
    CylinderParams cylinder;
    bool cylinder_found = false;

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr p_cloud_;
    pcl::PassThrough<pcl::PointXYZRGB> filterZ_;
    pcl::PassThrough<pcl::PointXYZRGB> filterX_;
    double filter_minZ = 0.0; // Distance from Camera
    double filter_maxZ = 0.4;
    double filter_minX = -0.2;
    double filter_maxX = 0.2;

    bool cup_added = false;
    bool first_call = true;
    std::string pc_topic = "/camera/depth/color/points";
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcmsg)
    {
        //RCLCPP_INFO(LOGGER2, "Perception Callback Begin");
        if(first_call)
        {
            // Convert Format
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            pcl::fromROSMsg(*pcmsg, *cloud_);
        
            // Filter Z
            filterZ_.setInputCloud(cloud_);
            filterZ_.setFilterFieldName("z");
            filterZ_.setFilterLimits(filter_minZ, filter_maxZ);
            filterZ_.filter(*cloud_);
            // Filter X(Y in World)
            filterX_.setInputCloud(cloud_);
            filterX_.setFilterFieldName("x");
            filterX_.setFilterLimits(filter_minX, filter_maxX);
            filterX_.filter(*cloud_);

            // Normals
            pcl::PointCloud<pcl::Normal>::Ptr normals_(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_(new pcl::search::KdTree<pcl::PointXYZRGB>());
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> estimation;
            
            estimation.setSearchMethod(tree_);
            estimation.setInputCloud(cloud_);
            estimation.setKSearch(50);
            estimation.compute(*normals_);

            // Remove Planar Surface
            pcl::PointIndices::Ptr inliers_plane_(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;

            segmentor.setOptimizeCoefficients(true);
            segmentor.setModelType(pcl::SACMODEL_PLANE);
            segmentor.setMethodType(pcl::SAC_RANSAC);
            segmentor.setMaxIterations(1000);
            segmentor.setDistanceThreshold(0.01);
            segmentor.setInputCloud(cloud_);

            pcl::ModelCoefficients::Ptr coeff_plane_(new pcl::ModelCoefficients);
            segmentor.segment(*inliers_plane_, *coeff_plane_);

            pcl::ExtractIndices<pcl::PointXYZRGB> extracted_indices;
            extracted_indices.setInputCloud(cloud_);
            extracted_indices.setIndices(inliers_plane_);
            extracted_indices.setNegative(true);
            extracted_indices.filter(*cloud_);

            // Extract Normals
            pcl::ExtractIndices<pcl::Normal> extracted;

            extracted.setNegative(true);
            extracted.setInputCloud(normals_);
            extracted.setIndices(inliers_plane_);
            extracted.filter(*normals_);

            // Extract Cylinder
            pcl::ModelCoefficients::Ptr coeff_cylinder_(new pcl::ModelCoefficients);
            pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segment;
            pcl::PointIndices::Ptr inliers_cylinder_(new pcl::PointIndices);

            segment.setOptimizeCoefficients(true);
            segment.setModelType(pcl::SACMODEL_CYLINDER);
            segment.setMethodType(pcl::SAC_RANSAC);
            segment.setNormalDistanceWeight(0.1);
            segment.setMaxIterations(10000);
            segment.setDistanceThreshold(0.05);  // Tolerance for Variation from Model
            segment.setRadiusLimits(0, 0.06); // Min/Max in Meters to Extract
            segment.setInputCloud(cloud_);
            segment.setInputNormals(normals_);
            segment.segment(*inliers_cylinder_, *coeff_cylinder_);

            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cloud_);
            extract.setIndices(inliers_cylinder_);
            extract.setNegative(false);
            extract.filter(*cloud_);

            if (cloud_->points.empty())
            {
            RCLCPP_ERROR_STREAM(LOGGER2, "Cylinder Not Found");
            rclcpp::shutdown();
            return;
            } else if(!cylinder_found){
                cylinder.radius = coeff_cylinder_->values[6];
                cylinder.orientation[0] = coeff_cylinder_->values[3];
                cylinder.orientation[1] = coeff_cylinder_->values[4];
                cylinder.orientation[2] = coeff_cylinder_->values[5];
                cylinder.cLine[0] = coeff_cylinder_->values[0];
                cylinder.cLine[1] = coeff_cylinder_->values[1];
                cylinder.cLine[2] = coeff_cylinder_->values[2];


                double max_angle_y = -std::numeric_limits<double>::infinity();
                double min_angle_y = std::numeric_limits<double>::infinity();
                double lowest_point[3] = { 0.0, 0.0, 0.0 };
                double highest_point[3] = { 0.0, 0.0, 0.0 };

                for (auto const point : cloud_->points)
                {
                    const double angle = atan2(point.z, point.y);
                    if (angle < min_angle_y)
                    {
                        min_angle_y = angle;
                        lowest_point[0] = point.x;
                        lowest_point[1] = point.y;
                        lowest_point[2] = point.z;
                    } else if (angle > max_angle_y){ 
                        max_angle_y = angle;
                        highest_point[0] = point.x;
                        highest_point[1] = point.y;
                        highest_point[2] = point.z;
                    }
                }
                
                cylinder.center[0] = (highest_point[0] + lowest_point[0]) / 2; 
                cylinder.center[1] = (highest_point[1] + lowest_point[1]) / 2;
                cylinder.center[2] = (highest_point[2] + lowest_point[2]) / 2;

                cylinder.height =
                    sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
                    pow((lowest_point[2] - highest_point[2]), 2));

                RCLCPP_INFO(LOGGER2, "CB Cup Radius %.3f, Cup Height %.3f, Cup X %.3f Cup Y %.3f Cup Z %.3f", 
                    cylinder.radius, cylinder.height, cylinder.center[0], cylinder.center[1], cylinder.center[2]);
                
                cylinder_found = true;
            }
            first_call = false;
        }
    } // END PC CALLBACK
}; // END PERCEPTION CLASS

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

moveit_msgs::msg::CollisionObject createCup(std::string name, const poses &cup_pose, const float &cup_height, const float &cup_radius)
{
    geometry_msgs::msg::Pose pose = toPose(cup_pose);
	moveit_msgs::msg::CollisionObject object;
	object.id = name;
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
    spawnObjects(psi, createCup("cup", cup_pose, cup_height, cup_radius));
    spawnObjects(psi, createCup("cup2", cup2_pose, cup_height, cup2_radius));
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
    //sampling_planner->setProperty("planning_attempts", 2);
    
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(0.001);
    cartesian_planner->setJumpThreshold(1.75);
    
    // Constraints 
    moveit_msgs::msg::OrientationConstraint gc;
    gc.link_name = "tool0"; 
    gc.header.frame_id = "world";
    gc.orientation.x = -0.70707; // 
    gc.orientation.y = 0.0;
    gc.orientation.z = 0.0;
    gc.orientation.w = 0.70714; 
    gc.absolute_x_axis_tolerance = 0.707;
    gc.absolute_y_axis_tolerance = 3.15;
    gc.absolute_z_axis_tolerance = 0.707;
    gc.weight = 1.0;

    moveit_msgs::msg::Constraints grasp_constraint;
    grasp_constraint.orientation_constraints.push_back(gc);

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
        stage->setTimeout(15.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->setPathConstraints(grasp_constraint);
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
            //stage->setPathConstraints(grasp_constraint);
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
            wrapper->setIKFrame(toEigen(grasp_frame_transform), gripper_frame); // Pose and Frame
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }
        /*
        // ***Allow Collision <PlanningScene>*** 
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
            stage->allowCollisions(
                "cup", 
                task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }    
        
        // ***Attach Cup <PlanningScene>***  
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cup");
            stage->attachObject("cup", gripper_frame);
            grasp->insert(std::move(stage));
        }
        */

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
        // ***Close Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("close");       
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
            auto stage = std::make_unique<mtc::stages::GeneratePose>("generate fill pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "fill_pose");
            //stage->setObject("cup");
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
            wrapper->setIKFrame(toEigen(grasp_frame_transform), gripper_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            fill->insert(std::move(wrapper));
        }

        // ***Retract from Coffee Machine <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retract coffee1", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.1, 0.2);
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
        // ***Open Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("open");
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
        stage->setPathConstraints(grasp_constraint);
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
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate regrasp position");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "regrasp_position");
            stage->setPreGraspPose("open");
            stage->setObject("cup2");
            stage->setAngleDelta(M_PI / 32); 
            // Define Pose 
            //geometry_msgs::msg::PoseStamped regrasp_position_msg;
            //regrasp_position_msg.header.frame_id = "world";
            //regrasp_position_msg.pose = toPose(regrasp_position);
            //stage->setPose(regrasp_position_msg);
            stage->setMonitoredStage(fill_stage_);

            // Compute IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("regrasp position IK", std::move(stage));
            wrapper->setMaxIKSolutions(20);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(toEigen(grasp_frame_transform2), gripper_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            regrasp->insert(std::move(wrapper));
        }

        // ***Lower Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lower cup", cartesian_planner);
            stage->properties().set("marker_ns", "lower_cup");
            //stage->properties().set("link", gripper_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(place_approach.min, place_approach.max);

            // Set down direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = place_approach.z; // ADJUST IF NEEDED 
            stage->setDirection(vec);
            regrasp->insert(std::move(stage));
        }


        /*
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
        */
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
        stage->setTimeout(15.0);
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
            auto stage = std::make_unique<mtc::stages::GeneratePose>("generate regrasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
			stage->properties().set("marker_ns", "regrasp_pose");
			//stage->setPreGraspPose("open");
			//stage->setObject("cup");
			//stage->setAngleDelta(M_PI / 16);
            geometry_msgs::msg::PoseStamped regrasp_position_msg;
            regrasp_position_msg.header.frame_id = "world";
            regrasp_position_msg.pose = toPose(regrasp_position);
            stage->setPose(regrasp_position_msg);
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
        /*
        // *** Allow Collision <PlanningScene>***
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
			stage->allowCollisions(
			    "cup",
			    task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			regrasp->insert(std::move(stage));
        }
        */
        // ***Close Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
			stage->setGroup(gripper_group_name);
			stage->setGoal("close");
			regrasp->insert(std::move(stage));    
        }
        /*
        // ***Attach Cup <PlanningScene>***
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cup");
			stage->attachObject("cup", gripper_frame);
			regrasp->insert(std::move(stage));
        }
        */

        // ***Lift Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cup", cartesian_planner);
			stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			stage->setIKFrame(gripper_frame);
			stage->properties().set("marker_ns", "lift_cup");

			// Set direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = "world";
			vec.vector.z = regrasp_retreat.z;
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
        stage->setTimeout(15.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        // Constraints 
        moveit_msgs::msg::OrientationConstraint sc;
        sc.link_name = "tool0"; 
        sc.header.frame_id = "world";
        sc.orientation.x = -1.0; // 
        sc.orientation.y = 0.0;
        sc.orientation.z = 0.0;
        sc.orientation.w = 0.0; 
        sc.absolute_x_axis_tolerance = 0.707;
        sc.absolute_y_axis_tolerance = 0.707;
        sc.absolute_z_axis_tolerance = 3.15;
        sc.weight = 1.0;

        moveit_msgs::msg::Constraints serve_constraint;
        serve_constraint.orientation_constraints.push_back(sc);
        stage->setPathConstraints(serve_constraint);
        task.add(std::move(stage));
    }
    // *** Serve Container ***
    {
        auto serve = std::make_unique<mtc::SerialContainer>("serve coffee");
        task.properties().exposeTo(serve->properties(), {"eef", "hand", "group", "ik_frame"});
        serve->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});
        
        // ***Serve to Baristabot <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("move to baristabot", cartesian_planner);
			stage->properties().set("marker_ns", "serve_cup");
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			
			// Set direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = "world";
			vec.vector.z = serve_approach.z; 
			stage->setDirection(vec);
			serve->insert(std::move(stage));
        }   
        // *** Serve Coffee Pose <Generator>***
        {
            auto stage = std::make_unique<mtc::stages::GeneratePose>("generate serve pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "serve_pose");
            //stage->setObject("cup");
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
            wrapper->setIKFrame(toEigen(regrasp_frame_transform), gripper_frame);
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
        /*
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
        */
        // ***Retract from Serve Pose
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("move to baristabot", cartesian_planner);
			stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			stage->setIKFrame(gripper_frame);
			stage->properties().set("marker_ns", "serve_cup");

			// Set direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = "world";
			vec.vector.z = serve_retreat.z; 
			stage->setDirection(vec);
			serve->insert(std::move(stage));
        }  
        task.add(std::move(serve));
    } // END SERVE CONTAINER
        // ***Move to Pre-Serve Position <MoveRelative>***
        /*
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("pre-serve1", cartesian_planner);
			stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			stage->setIKFrame(gripper_frame);
			stage->properties().set("marker_ns", "preserve1");

			// Set direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = "world";
			vec.vector.x = preserve1.x;
            vec.vector.y = preserve1.y;
			stage->setDirection(vec);
			regrasp->insert(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("pre-serve2", cartesian_planner);
			stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			stage->setIKFrame(gripper_frame);
			stage->properties().set("marker_ns", "preserve2");

			// Set direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = "world";
			vec.vector.x = preserve2.x;
            vec.vector.y = preserve2.y;
			stage->setDirection(vec);
			regrasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("pre-serve1", cartesian_planner);
			stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			stage->setIKFrame(gripper_frame);
			stage->properties().set("marker_ns", "preserve1");

			// Set direction
			geometry_msgs::msg::TwistStamped twist;
			twist.header.frame_id = "world";
            twist.twist.angular.z = M_PI / 3;
			stage->setDirection(twist);
			regrasp->insert(std::move(stage));
        }
        */

        


    // RETURN HOME
    /*
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal("home");
        stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
        task.add(std::move(stage));
    }
    */
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
        rclcpp::sleep_for(std::chrono::milliseconds(7500));
    }
    perception_node->addCup();
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
    mtc_node->setupPlanningScene();

    // Wait for Octomap Update
    std::cout << "Waiting for Octomap Update" << std::endl;
    rclcpp::sleep_for(std::chrono::milliseconds(2000));   
    mtc_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();

    return 0;
} // END MAIN

