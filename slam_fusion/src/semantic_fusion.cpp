/**
 * @file semantic_fusion.cpp
 * @brief Semantic 3D Fusion Node — ROS 2 Humble
 *
 * Fuses 2D semantic masks, depth images, and odometry to construct a 
 * globally consistent, colorized 3D point cloud map.
 *
 * Subscribes to:
 *   - /sam2/semantic_mask (Instance IDs)
 *   - /camera/depth/image_raw (Depth data)
 *   - /slam/odom (Camera poses)
 *   - /camera/color/camera_info (Intrinsics)
 *
 * Publishes:
 *   - /semantic_map (Colorized PointCloud2)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <tuple>

class SemanticFusionNode : public rclcpp::Node
{
public:
    SemanticFusionNode()
    : Node("semantic_fusion_node")
    {
        // ── ROS Parameters ────────────────────────────────────────────────
        this->declare_parameter<int>("pixel_step",        2);      // down-sample step
        this->declare_parameter<float>("depth_min",       0.1f);   // metres
        this->declare_parameter<float>("depth_max",       10.0f);  // metres
        this->declare_parameter<float>("voxel_leaf_size", 0.05f);  // metres (0 = disabled)
        this->declare_parameter<int>("sync_queue_size",   20);

        pixel_step_      = this->get_parameter("pixel_step").as_int();
        depth_min_       = static_cast<float>(this->get_parameter("depth_min").as_double());
        depth_max_       = static_cast<float>(this->get_parameter("depth_max").as_double());
        voxel_leaf_size_ = static_cast<float>(this->get_parameter("voxel_leaf_size").as_double());
        int q            = this->get_parameter("sync_queue_size").as_int();

        // Accumulated global semantic map (persists across callbacks)
        global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        // ── TF Initialization ─────────────────────────────────────────────
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        // ── Publishers ────────────────────────────────────────────────────
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/semantic_map", 10);

        // ── Camera Info Subscriber ────────────────────────────────────────
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 10,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                latest_info_ = msg;
            });

        // ── Synchronised Subscribers ──────────────────────────────────────
        sub_mask_.subscribe(this, "/sam2/semantic_mask");
        sub_depth_.subscribe(this, "/camera/depth/image_raw");
        sub_odom_.subscribe(this, "/slam/odom");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(q), sub_mask_, sub_depth_, sub_odom_);

        sync_->registerCallback(
            std::bind(&SemanticFusionNode::fusionCallback, this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3));

        RCLCPP_INFO(this->get_logger(),
            "Semantic Fusion Node ready | pixel_step=%d | depth [%.2f, %.2f] m | "
            "voxel_leaf=%.3f m",
            pixel_step_, depth_min_, depth_max_, voxel_leaf_size_);
    }

private:
    // ── Member types ──────────────────────────────────────────────────────
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,       // mask
        sensor_msgs::msg::Image,       // depth
        nav_msgs::msg::Odometry>;      // pose

    // ── Members ───────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    sensor_msgs::msg::CameraInfo::SharedPtr latest_info_;

    message_filters::Subscriber<sensor_msgs::msg::Image>     sub_mask_;
    message_filters::Subscriber<sensor_msgs::msg::Image>     sub_depth_;
    message_filters::Subscriber<nav_msgs::msg::Odometry>     sub_odom_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;

    int   pixel_step_;
    float depth_min_;
    float depth_max_;
    float voxel_leaf_size_;

    // ── Helpers ───────────────────────────────────────────────────────────

    /// Maps a SAM2 instance ID (1-254) to a repeatable, visually distinct RGB.
    static std::tuple<uint8_t, uint8_t, uint8_t> idToColor(uint8_t id)
    {
        if (id == 0) return {0, 0, 0};  // background = black
        uint8_t r = static_cast<uint8_t>((id * 123u) % 255u);
        uint8_t g = static_cast<uint8_t>((id *  73u) % 255u);
        uint8_t b = static_cast<uint8_t>((id * 201u) % 255u);
        return {r, g, b};
    }

    // ── Main Fusion Callback ──────────────────────────────────────────────
    void fusionCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr & mask_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
        const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
    {
        auto t0 = std::chrono::steady_clock::now();

        if (!latest_info_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for CameraInfo…");
            return;
        }

        // Convert incoming ROS messages to OpenCV format
        cv_bridge::CvImagePtr cv_depth, cv_mask;
        try {
            cv_depth = cv_bridge::toCvCopy(depth_msg,
                           sensor_msgs::image_encodings::TYPE_32FC1);
            cv_mask  = cv_bridge::toCvCopy(mask_msg,
                           sensor_msgs::image_encodings::MONO8);
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        // Retrieve camera intrinsics for projection
        const float fx = static_cast<float>(latest_info_->k[0]);
        const float cx = static_cast<float>(latest_info_->k[2]);
        const float fy = static_cast<float>(latest_info_->k[4]);
        const float cy = static_cast<float>(latest_info_->k[5]);

        // Initialize local point cloud in the camera coordinate frame
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>());
        local_cloud->reserve(
            (cv_mask->image.rows / pixel_step_) *
            (cv_mask->image.cols / pixel_step_));

        for (int v = 0; v < cv_mask->image.rows; v += pixel_step_) {
            for (int u = 0; u < cv_mask->image.cols; u += pixel_step_) {
                uint8_t class_id = cv_mask->image.at<uint8_t>(v, u);
                float   z        = cv_depth->image.at<float>(v, u);

                // Filter out invalid or out-of-range pixels
                if (class_id == 0 || std::isnan(z) || z < depth_min_ || z > depth_max_)
                    continue;

                // Project 2D pixel coordinates to 3D space
                pcl::PointXYZRGB pt;
                pt.z = z;
                pt.x = (static_cast<float>(u) - cx) * z / fx;
                pt.y = (static_cast<float>(v) - cy) * z / fy;

                auto [r, g, b] = idToColor(class_id);
                pt.r = r;  pt.g = g;  pt.b = b;

                local_cloud->push_back(pt);
            }
        }

        // Calculate Sensor to Base Transform
        std::string child_frame = odom_msg->child_frame_id;
        if (child_frame.empty()) {
            child_frame = depth_msg->header.frame_id; // Default to sensor frame if unspecified
        }

        geometry_msgs::msg::TransformStamped t_sensor_to_base;
        try {
            // Retrieve spatial transform from TF tree
            t_sensor_to_base = tf2_buffer_->lookupTransform(
                child_frame,                 // Target frame
                depth_msg->header.frame_id,  // Source frame
                tf2_ros::fromMsg(depth_msg->header.stamp), // Time
                tf2::Duration(std::chrono::milliseconds(100))); // Timeout
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup failed: %s", ex.what());
            return;
        }

        // Convert to Eigen
        Eigen::Isometry3f T_sensor_to_base = tf2::transformToEigen(t_sensor_to_base.transform).cast<float>();

        // Calculate Base to World Transform from Odometry
        Eigen::Quaternionf q(
            static_cast<float>(odom_msg->pose.pose.orientation.w),
            static_cast<float>(odom_msg->pose.pose.orientation.x),
            static_cast<float>(odom_msg->pose.pose.orientation.y),
            static_cast<float>(odom_msg->pose.pose.orientation.z));
        Eigen::Vector3f t(
            static_cast<float>(odom_msg->pose.pose.position.x),
            static_cast<float>(odom_msg->pose.pose.position.y),
            static_cast<float>(odom_msg->pose.pose.position.z));

        Eigen::Isometry3f T_base_to_world = Eigen::Isometry3f::Identity();
        T_base_to_world.linear() = q.toRotationMatrix();
        T_base_to_world.translation() = t;

        // Compute final World Transform
        Eigen::Matrix4f T_total = (T_base_to_world * T_sensor_to_base).matrix();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_world(
            new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*local_cloud, *frame_world, T_total);

        // Merge current frame into the global map
        *global_map_ += *frame_world;

        // Apply Voxel-Grid filter to maintain real-time publishing performance
        if (voxel_leaf_size_ > 0.0f && !global_map_->empty()) {
            pcl::VoxelGrid<pcl::PointXYZRGB> vg;
            vg.setInputCloud(global_map_);
            vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(
                new pcl::PointCloud<pcl::PointXYZRGB>());
            vg.filter(*filtered);
            global_map_ = filtered;
        }

        // Publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*global_map_, output_msg);
        output_msg.header.frame_id = "odom";
        output_msg.header.stamp    = odom_msg->header.stamp;
        pc_pub_->publish(output_msg);

        // Latency log
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Fusion: local=%zu pts | global=%zu pts | latency=%ld ms",
            local_cloud->size(), global_map_->size(), elapsed_ms);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticFusionNode>());
    rclcpp::shutdown();
    return 0;
}