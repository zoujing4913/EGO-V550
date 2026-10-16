//
// Created by Hyungtae Lim on 6/23/21.
// Ported to ROS 2 (Humble) and extended with frame handling.
//
// patchwork splits a point cloud into ground and non-ground points.
//
// IMPORTANT about frames: patchwork's concentric zone model computes the radius
// and the height of every point with respect to the *origin of the cloud*, i.e.
// it expects a cloud in the sensor frame. Its inputs, however, are often
// registered/accumulated clouds expressed in the map frame (that is what the
// planner wants). Therefore:
//   * `sensor_frame`  (default: empty)  - the cloud is transformed into this
//     frame before the segmentation (e.g. "lidar_link"),
//   * `output_frame`  (default: empty)  - ground/non-ground clouds are published
//     in this frame (e.g. "map"); empty means "keep the frame of the input".
// Without `sensor_frame` the points are segmented in whatever frame they arrive
// in; if that is far away from the sensor (e.g. a map frame where the robot is
// 60 m from the origin) *every* point falls outside max_r and nothing is
// classified at all.

// For disable PCL complile lib, to use PointXYZILID
#define PCL_NO_PRECOMPILE
#include "patchwork/patchwork.hpp"
#include <cstdlib>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>


using PointType = pcl::PointXYZ;
using namespace std;

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr CloudPublisher;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PositivePublisher;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr NegativePublisher;

pcl::PointCloud<PointType> pc_curr;
pcl::PointCloud<PointType> pc_ground;
pcl::PointCloud<PointType> pc_non_ground;
static double time_taken;

boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

static std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::string sensor_frame_;   // segment in this frame ("" = keep input frame)
std::string output_frame_;   // publish in this frame ("" = keep input frame)
bool allow_latest_tf_{true}; // simulation compatibility; real robot uses exact-time TF only


template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(const sensor_msgs::msg::PointCloud2 &cloudmsg)
{
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg,cloudresult);
    return cloudresult;
}

template<typename T>
sensor_msgs::msg::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const std::string & frame_id)
{
    sensor_msgs::msg::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

// transform a cloud into `target` frame; returns false if no transform is available
static bool transformCloud(const sensor_msgs::msg::PointCloud2 &in, const std::string &target,
                           sensor_msgs::msg::PointCloud2 &out, const std::string &what)
{
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(target, in.header.frame_id, in.header.stamp,
                                         rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &ex) {
        if (!allow_latest_tf_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "cannot transform %s at measurement time: %s",
                                 what.c_str(), ex.what());
            return false;
        }
        // the odometry TF of the robot can be a few milliseconds younger than the
        // scan stamp, so fall back to the latest available transform
        try {
            tf = tf_buffer_->lookupTransform(target, in.header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "cannot transform %s ('%s' -> '%s'): %s",
                                 what.c_str(), in.header.frame_id.c_str(), target.c_str(), ex.what());
            return false;
        }
    }
    tf2::doTransform(in, out, tf);
    out.header.stamp = in.header.stamp;
    out.header.frame_id = target;
    return true;
}

static void publishCloud(const pcl::PointCloud<PointType> &cloud,
                         const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
                         const sensor_msgs::msg::PointCloud2 &reference)
{
    const std::string segment_frame = reference.header.frame_id;
    const std::string frame = output_frame_.empty() ? segment_frame : output_frame_;
    auto cloud_msg = cloud2msg(cloud, segment_frame);
    cloud_msg.header.stamp = reference.header.stamp;

    if (frame == segment_frame) {
        pub->publish(cloud_msg);
        return;
    }
    sensor_msgs::msg::PointCloud2 out;
    if (transformCloud(cloud_msg, frame, out, "segmented cloud")) {
        pub->publish(out);
    }
}

void callbackNode(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    sensor_msgs::msg::PointCloud2 cloud_segmentation;
    if (!sensor_frame_.empty() && msg->header.frame_id != sensor_frame_) {
        if (!transformCloud(*msg, sensor_frame_, cloud_segmentation, "cloud into the sensor frame")) {
            return;
        }
    } else {
        cloud_segmentation = *msg;
    }

    pcl::fromROSMsg(cloud_segmentation, pc_curr);
    PatchworkGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

    publishCloud(pc_curr, CloudPublisher, cloud_segmentation);
    publishCloud(pc_ground, PositivePublisher, cloud_segmentation);
    publishCloud(pc_non_ground, NegativePublisher, cloud_segmentation);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    node_ = std::make_shared<rclcpp::Node>("ground_segmentation");

    // frames of the segmentation / of the published clouds
    sensor_frame_ = node_->declare_parameter<std::string>("sensor_frame", "");
    output_frame_ = node_->declare_parameter<std::string>("output_frame", "");
    allow_latest_tf_ = node_->declare_parameter<bool>("allow_latest_tf", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);

    PatchworkGroundSeg.reset(new PatchWork<PointType>(node_));
    // the patch polygons are computed in the segmented frame
    PatchworkGroundSeg->setVisualizationFrame(
        sensor_frame_.empty() ? "map" : sensor_frame_);

    CloudPublisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/cloud", 100);
    PositivePublisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/P", 100);
    NegativePublisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/N", 100);

    auto cloud_sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud_scan", rclcpp::SensorDataQoS(), callbackNode);

    RCLCPP_INFO(node_->get_logger(),
                "ground segmentation ready (segment frame '%s', output frame '%s')",
                sensor_frame_.empty() ? "<input frame>" : sensor_frame_.c_str(),
                output_frame_.empty() ? "<input frame>" : output_frame_.c_str());

    cout << "Operating patchwork..." << endl;

    rclcpp::spin(node_);
    rclcpp::shutdown();

    return 0;
}
