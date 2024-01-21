#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <cmath>

static const rclcpp::Logger LOGGER2 = rclcpp::get_logger("perception");


class Perception : public rclcpp::Node
{
using std::placeholders::_1;

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
        /* 
         * NOTE: Using Known Camera Frame Orientation 
         * This forces the Z-axis of the cup object to be up.
         * Otherwise the orientation of the Z-axis appears to be random between up and down.
        */
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
    // Distance from Camera
    double filter_minZ = 0.0; 
    double filter_maxZ = 0.4;
    double filter_minX = -0.2;
    double filter_maxX = 0.2;
    // Adjustment for Cup X-Axis Rotation (Radians)
    double cup_offset = -0.0855;
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