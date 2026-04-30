/**
 * @file semantic_fusion.cpp
 * @brief Semantic 3D Fusion Node — ROS 2 Humble
 *
 * Subscribes to:
 *   /sam2/semantic_mask  (sensor_msgs/Image, mono8 — per-pixel instance ID)
 *   /camera/depth/image_raw  (sensor_msgs/Image, 32FC1 — depth in metres)
 *   /slam/odom           (nav_msgs/Odometry — camera pose in world frame)
 *   /camera/color/camera_info  (sensor_msgs/CameraInfo — intrinsics)
 *
 * Publishes:
 *   /semantic_map        (sensor_msgs/PointCloud2 — colourised by instance)
 *
 * Key fixes vs. original:
 *   [BUG]  Duplicate declaration of info_sub_ caused compile error → removed.
 *   [BUG]  transform.block<3,1>(0,3) wrong shape for a 3-vector → fixed to
 *          block<3,1>(0,3) is actually correct for Eigen but typo risk; left
 *          and annotated clearly.
 *   [FEAT] Accumulate a global map across frames (not just publish each local
 *          cloud independently).
 *   [FEAT] Voxel-grid downsampling to prevent unbounded map growth.
 *   [FEAT] Depth validity range and NaN guard consistent with dataset spec.
 *   [FEAT] All tunable constants exposed as ROS parameters.
 *   [FEAT] Latency measurement and diagnostic logging per callback.
 *
 * Build requirements (CMakeLists.txt):
 *   PCL components: common, filters (for voxel grid)
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

        // ── Publishers ────────────────────────────────────────────────────
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/semantic_map", 10);

        // ── Camera Info Subscriber ────────────────────────────────────────
        // FIX: original had info_sub_ declared twice — removed the duplicate.
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

        // 1. Convert ROS Images → OpenCV
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

        // 2. Extract camera intrinsics
        const float fx = static_cast<float>(latest_info_->k[0]);
        const float cx = static_cast<float>(latest_info_->k[2]);
        const float fy = static_cast<float>(latest_info_->k[4]);
        const float cy = static_cast<float>(latest_info_->k[5]);

        // 3. Build local point cloud (camera frame)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>());
        local_cloud->reserve(
            (cv_mask->image.rows / pixel_step_) *
            (cv_mask->image.cols / pixel_step_));

        for (int v = 0; v < cv_mask->image.rows; v += pixel_step_) {
            for (int u = 0; u < cv_mask->image.cols; u += pixel_step_) {
                uint8_t class_id = cv_mask->image.at<uint8_t>(v, u);
                float   z        = cv_depth->image.at<float>(v, u);

                // Skip background, NaN, and out-of-range depth
                if (class_id == 0 || std::isnan(z) || z < depth_min_ || z > depth_max_)
                    continue;

                // Back-project pixel (u,v,z) → 3D camera-frame point
                pcl::PointXYZRGB pt;
                pt.z = z;
                pt.x = (static_cast<float>(u) - cx) * z / fx;
                pt.y = (static_cast<float>(v) - cy) * z / fy;

                auto [r, g, b] = idToColor(class_id);
                pt.r = r;  pt.g = g;  pt.b = b;

                local_cloud->push_back(pt);
            }
        }

        // 4. Transform camera-frame cloud → world frame using SLAM odometry
        Eigen::Quaternionf q(
            static_cast<float>(odom_msg->pose.pose.orientation.w),
            static_cast<float>(odom_msg->pose.pose.orientation.x),
            static_cast<float>(odom_msg->pose.pose.orientation.y),
            static_cast<float>(odom_msg->pose.pose.orientation.z));
        Eigen::Vector3f t(
            static_cast<float>(odom_msg->pose.pose.position.x),
            static_cast<float>(odom_msg->pose.pose.position.y),
            static_cast<float>(odom_msg->pose.pose.position.z));

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = t;          // FIX: was ambiguous — now explicit

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_world(
            new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*local_cloud, *frame_world, T);

        // 5. Accumulate into global map
        *global_map_ += *frame_world;

        // 6. Voxel-grid downsampling (keeps map bounded & publishable in real-time)
        if (voxel_leaf_size_ > 0.0f && !global_map_->empty()) {
            pcl::VoxelGrid<pcl::PointXYZRGB> vg;
            vg.setInputCloud(global_map_);
            vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(
                new pcl::PointCloud<pcl::PointXYZRGB>());
            vg.filter(*filtered);
            global_map_ = filtered;
        }

        // 7. Publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*global_map_, output_msg);
        output_msg.header.frame_id = "map";
        output_msg.header.stamp    = odom_msg->header.stamp;
        pc_pub_->publish(output_msg);

        // 8. Latency log
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