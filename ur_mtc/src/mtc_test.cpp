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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_control");
static const rclcpp::Logger LOGGER2 = rclcpp::get_logger("perception");

namespace mtc = moveit::task_constructor;
using std::placeholders::_1;

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
// TODO: Add Planning Scene Functions Here

class Perception : public rclcpp::Node
{
public:
    Perception() : Node("perception")
    {
        this->p_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    pc_topic, 10, std::bind(&Perception::cloudCallback, this, _1));
        RCLCPP_INFO(LOGGER2,"Node Created");
    }
    
    void addobject()
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::CollisionObject object;
        geometry_msgs::msg::Pose pose;
        //Eigen::Vector3d object_z(cylinder.orientation[0], cylinder.orientation[1], cylinder.orientation[2]);
        //Eigen::Vector3d world_z(0.0, 0.0, 1.0);
        //Eigen::Vector3d axis = object_z.cross(world_z);
        //axis.normalize();
        //double angle = acos(world_z.dot(object_z));
        // Using Known Camera Frame Orientation
        
        object.id = "object";
        object.header.frame_id = "wrist_rgbd_camera_depth_optical_frame";
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	    object.primitives[0].dimensions = { cylinder.height, cylinder.radius };
        pose.orientation.x = 2.35619; //axis.x() * sin(angle / 2);
        pose.orientation.y = 0.0; //axis.y() * sin(angle / 2);
        pose.orientation.z = 0.0; //axis.z() * sin(angle / 2);
        // RCLCPP_INFO(LOGGER, "Rot X: %f, Rot Y: %f, Rot Z: %f, Angle: %f", pose.orientation.x, pose.orientation.y, pose.orientation.z, angle);
        pose.orientation.w = 1.0;
        pose.position.x = cylinder.center[0];
        pose.position.y = cylinder.center[1];
        pose.position.z = cylinder.center[2];
        object.primitive_poses.push_back(pose);

        psi.applyCollisionObject(object);
        RCLCPP_INFO(LOGGER2,"object Created");
        object_added = true;
    }

    struct CylinderParams{
        double radius;
        double orientation[3];
        double center[3];
        double height;
        double cLine[3];
    };
    CylinderParams cylinder;

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr p_cloud_;
    pcl::PassThrough<pcl::PointXYZRGB> filterZ_;
    pcl::PassThrough<pcl::PointXYZRGB> filterX_;
    double filter_minZ = 0.0; // Distance from Camera
    double filter_maxZ = 0.75;
    double filter_minX = -0.2;
    double filter_maxX = 0.2;
    bool cylinder_found = false;
    bool object_added = false;
    std::string pc_topic = "/wrist_rgbd_depth_sensor/points";
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcmsg)
    {
        if(!object_added)
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

                RCLCPP_INFO(LOGGER, "object Radius %.3f, object Height %.3f, object X %.3f object Y %.3f object Z %.3f", 
                    cylinder.radius, cylinder.height, cylinder.center[0], cylinder.center[1], cylinder.center[2]);
                
                cylinder_found = true;
            }
        }
    } // END PC CALLBACK
}; // END PERCEPTION CLASS

class Control
{
private:
    mtc::TaskPtr task_;

    // Direction Arrays X, Y, Z, Min, Max  Use 1.0 in axis direction for min/max with non-zero min/max values
    // TODO: Add Real Values
    float grasp_approach[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; 
    float grasp_lift[5] = {0.0, 0.0, 0.0, 0.0, 0.0};    
    float place_lower[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    float place_retreat[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    struct poses{
        float x;
        float y;
        float z;
        float rx;
        float py;
        float yz;      
    } grasp_transform, place_pose;

    grasp_trasform = {0.0, 0.02, 0.2, M_PI, 0.0, 0.0};
    place_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

    void taskReset()
    {
        task_.reset();
	    task_.reset(new mtc::Task());
    }

    void initialState()
    {
        auto current_state = std::make_unique<stages::CurrentState>("initial state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([object = params.object_name](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object with id '" + object + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});
		task.add(std::move(applicability_filter));
    }

    void goHome()
    {
        auto stage = std::make_unique<stages::MoveTo>("go to home", sampling_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGoal("home");
		stage->restrictDirection(stages::MoveTo::FORWARD);
		task.add(std::move(stage));
    }

    void activateGripper(std::unique_ptr<mtc::SerialContainer> &container = nullptr, const std::string pose)
    {
        std::string stage_name = pose + " gripper";
        auto stage = std::make_unique<stages::MoveTo>(stage_name, interpolation_planner);
		stage->setGroup(gripper_group_name);
		stage->setGoal(pose);
        if(container)
        {
            container->insert(std::move(stage));
        }else{
            task.add(std::move(stage));
        }
    }

    void connectStage(const std::string stage_name, const float timeout)
    {
        auto stage = std::make_unique<stages::Connect>(
		    stage_name, stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
		stage->setTimeout(timeout);
		stage->properties().configureInitFrom(Stage::PARENT);
		task.add(std::move(stage));
    }

    void moveRelative(std::unique_ptr<mtc::SerialContainer> &container, const std::string stage_name, std::string frame, float direction[5], bool approach = false)
    {
        auto stage = std::make_unique<stages::MoveRelative>(stage_name, cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		if(direction[4] > 0)
        {
            stage->setMinMaxDistance(direction[3], direction[4]);
        }
        if(!approach)
        {
            stage->setIKFrame(gripper_frame);
        }
		stage->properties().set("marker_ns", stage_name);

		geometry_msgs::msg::Vector3Stamped vec;
		vec.header.frame_id = frame;
		vec.vector.x = direction[0];
        vec.vector.y = direction[1];
        vec.vector.z = direction[2];
		stage->setDirection(vec);    

        if(container)
        {
            container->insert(std::move(stage));
        }else{
            task.add(std::move(stage));
        }
    }

    void generateGraspPose(std::unique_ptr<mtc::SerialContainer> &container, const std::string stage_name, mtc::Stage &stage_ptr, poses &transform, const std::string object, const std::string pre_pose = "open" )
    {
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>(stage_name);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", stage_name); 
            stage->setPreGraspPose(pre_pose);
            stage->setObject(object);
            stage->setAngleDelta(M_PI / 16); 
            stage->setMonitoredStage(stage_ptr);
            
            // ***Compute IK <Wrapper>***
            std::string ik_name = stage_name + " IK";
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>(ik_name, std::move(stage));
            wrapper->setMaxIKSolutions(12);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(toEigen(transform), gripper_frame); // Pose and Frame
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            container->insert(std::move(wrapper));
    }
    
    void generatePlacePose(std::unique_ptr<mtc::SerialContainer> &container, const std::string stage_name, const std::string frame, mtc::Stage &stage_ptr, poses &pose, const std::string object)
    {
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>(stage_name);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", stage_name);
        stage->setObject(object);
        // Define Pose 
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame;
        pose_msg.pose = toPose(pose);
        stage->setPose(pose_msg);
        stage->setMonitoredStage(stage_ptr);

        // Compute IK
        std::string ik_name = stage_name + " IK";
        auto wrapper = std::make_unique<mtc::stages::ComputeIK>(ik_name, std::move(stage));
        wrapper->setMaxIKSolutions(12);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(object);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        container->insert(std::move(wrapper));
    }
    void enableCollision(std::unique_ptr<mtc::SerialContainer> &container, const std::string object, const bool enable)
    {
        std::string stage_name;
        if(enable){
            stage_name = "allow collision";
        }else{
            stage_name = "disable collision";
        }
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(stage_name);
        stage->allowCollisions(
            object, 
            task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
            enable);
        container->insert(std::move(stage));
    }

    void affixObject(std::unique_ptr<mtc::SerialContainer> &container, const std::string object, const bool attach)
    {
        std::string stage_name;
        if(attach){
            stage_name = "attach object";
        } else {
            stage_name = "detach object";
        }
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(stage_name);
        if(attach)
        {
            stage->attachObject(object, gripper_frame);
        } else {
            stage->detachObject(object, gripper_frame);
        }
        container->insert(std::move(stage));
    }
public:
    Control()
    {
    }
    
    bool planTask(int max_solutions)
    {
        RCLCPP_INFO(LOGGER, "Begin Task Planning")
        if (task_.plan(max_solutions))
        {
            
            RCLCPP_INFO(LOGGER, "Task planning succeded");
            
        } else {
            RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
            return false;
        }
        task_.introspection().publishSolution(*task_.solutions().front());
        
        

        return true;
    }

    bool executeTask()
    {
        moveit_msgs::msg::MoveItErrorCodes result = task_->.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) 
        {
		    RCLCPP_ERROR_STREAM(LOGGER, "Execution Failed. Error Returned: " << result.val);
		    return false;
	    }

	    return true;
    }

    bool createTask(const rclcpp::Node::SharedPtr& node)
    {
        RCLCPP_INFO(LOGGER, "Initializing Task");
        // Reset Introspection / RVIZ
        taskReset();

        // Initialize Task and Planners
        mtc::Task &task = *task_;
        task.stages()->setName("Coffee PnP");
        task.loadRobotModel(node);

        const auto& arm_group_name = "ur_manipulator";
        const auto& gripper_group_name = "gripper";
        const auto& gripper_frame "tool0"; 

        task.setProperty("group", arm_group_name);
        task.setProperty("eef", gripper_group_name); 
        task.setProperty("hand", gripper_group_name);
        task.setProperty("hand_grasping_frame", gripper_frame);
        task.setProperty("ik_frame", gripper_frame); 

        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
        sampling_planner->setProperty("goal_joint_tolerance", 1e-4);
        
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(0.001);
        cartesian_planner->setJumpThreshold(1.5);

        // *** Stages ***
        mtc::Stage* initial_state = nullptr;
        initialState();

        // *** Close and Open Gripper ***
        activateGripper("close");
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("open"); 
            // Set Stage Pointer 
            initial_state = stage.get();
            task.add(std::move(stage));
        }
        // *** Move to Pre-Grasp Position ***
        connectStage("pre-grasp position", 15.0);

        mtc::Stage* grasp_stage = nullptr;
        // Grasp Container
        {
            auto grasp = std::make_unique<mtc::SerialContainer>("grasp");
            // Expose Properties
            task.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
            grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

            // *** Approach Object ***
            moveRelative(grasp, "approach object", gripper_frame, grasp_approach);

            // *** Generate Grasp Pose ***
            generateGraspPose(grasp, "generate grasp pose", initial_state, grasp_transform, object);

            // *** Allow Collisions ***
            enableCollision(grasp, object, true);

            // *** Close Gripper ***
            activateGripper(grasp, "close");

            // *** Attach Object ***
            affixObject(grasp, object, true);

            // *** Lift Object ***
            moveRelative(grasp, "lift object", "world", grasp_lift);

            // Set Stage Pointer
            grasp_stage = grasp.get();
            task.add(std::move(grasp));
        } // END GRASP CONTAINER

        // *** Move to Place ***
        connectStage("move to place", 15.0);

        // Place Container
        mtc::Stage* place_stage = nullptr;
        {
            auto place = std::make_unique<mtc::SerialContainer>("place object");
            task.properties().exposeTo(fill->properties(), {"eef", "hand", "group", "ik_frame"});
            place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

            // *** Place Object ***
            moveRelative(place, "lower object", "world", place_lower);
            
            // *** Generate Place Pose ***
            generatePlacePose(place, "generate place pose", grasp_stage, "world", place_pose, object);

            // *** Open Gripper ***
            activateGripper(place, "open");

            // *** Disable Collision ***
            enableCollision(place, object, false);
            
            // *** Detach Object ***
            affixObject(place, object, false);

            // *** Retreat from Object ***
            moveRelative(place, "retreat from object", "world", place_retreat);

            task.add(std::move(place));
        } // END PLACE CONTAINER

        // Move to Home
        goHome();

        // Initialize for Planning
        try {
		    task.init();
	    } catch (mtc::InitStageException& e) {
		    RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
		    return false;
	    }

	    return true;
    } // END CREATE TASK

}; // END CONTROL CLASS