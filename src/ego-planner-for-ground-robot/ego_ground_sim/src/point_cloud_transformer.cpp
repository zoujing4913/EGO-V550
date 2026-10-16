// Transforms the simulated 3D LiDAR scan into the planning frame.
//
// The Gazebo ray sensor publishes the cloud in the *sensor* frame (lidar_link).
// The planner, however, consumes the cloud as if it were expressed in the same
// frame as the odometry (the planning/map frame) - it never does a TF lookup
// itself. Therefore the points are really transformed here with tf2 (rotation +
// translation via tf2::doTransform), and only then published as
// /registered_scan. Changing header.frame_id alone would be wrong.
//
// Also filters non-finite points and points outside [min_range, max_range]
// (measured in the sensor frame, so self-hits and far noise disappear).

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace ego_ground_sim
{

class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer()
  : rclcpp::Node("point_cloud_transformer")
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/scan_3d");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/registered_scan");
    min_range_ = this->declare_parameter<double>("min_range", 0.2);
    max_range_ = this->declare_parameter<double>("max_range", 12.0);
    tf_timeout_ = this->declare_parameter<double>("tf_timeout", 0.05);
    max_tf_age_ = this->declare_parameter<double>("max_tf_age", 0.3);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/state_estimation");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    // Published reliably so that RViz2 / ros2 topic echo (which default to
    // reliable) can display it; best effort subscribers still work.
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(10));
    auto qos = rclcpp::SensorDataQoS();
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&PointCloudTransformer::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&PointCloudTransformer::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "transforming '%s' (%s frame) -> '%s' in '%s' frame",
      input_topic_.c_str(), "<cloud header>", output_topic_.c_str(), target_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    newest_state_stamp_ = rclcpp::Time(msg->header.stamp);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (msg->width * msg->height == 0) {
      return;
    }

    // 1) filter in the sensor frame: the range w.r.t. the sensor origin is
    //    invariant under the following rigid transform.
    std::vector<Eigen::Vector3f> points;
    points.reserve(msg->width * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float x = *it_x, y = *it_y, z = *it_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const double r = std::sqrt(x * x + y * y + z * z);
      if (r < min_range_ || r > max_range_) {
        continue;
      }
      points.emplace_back(x, y, z);
    }

    if (points.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "no valid point in the incoming scan");
      return;
    }
    sensor_msgs::msg::PointCloud2 filtered;
    filtered.header = msg->header;  // sensor frame, original stamp
    sensor_msgs::PointCloud2Modifier modifier(filtered);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    {
      sensor_msgs::PointCloud2Iterator<float> it_out_x(filtered, "x");
      sensor_msgs::PointCloud2Iterator<float> it_out_y(filtered, "y");
      sensor_msgs::PointCloud2Iterator<float> it_out_z(filtered, "z");
      for (const auto & p : points) {
        *it_out_x = p.x();
        *it_out_y = p.y();
        *it_out_z = p.z();
        ++it_out_x;
        ++it_out_y;
        ++it_out_z;
      }
    }

    // 2) real point-wise transform into the planning frame
    const rclcpp::Time scan_stamp(msg->header.stamp, this->get_clock()->get_clock_type());
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp,
                                       rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      // The odometry TF of the chassis can be a few milliseconds younger than
      // the scan stamp (sensor vs. physics update), which makes a lookup at the
      // exact scan time fail. Fall back to the latest available transform, but
      // only as long as it is fresh - otherwise a stale pose would distort the
      // mapped points.
      const double state_age = (scan_stamp - newest_state_stamp_).seconds();
      if (newest_state_stamp_.nanoseconds() == 0 || state_age > max_tf_age_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "cannot transform '%s' -> '%s': %s, and the newest state is %.3fs old "
          "(no scan published)", msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what(),
          state_age);
        return;
      }
      try {
        tf = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero);
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "using the latest available transform for '%s' -> '%s' (scan stamp %.3fs ahead)",
          msg->header.frame_id.c_str(), target_frame_.c_str(), state_age);
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "cannot transform '%s' -> '%s': %s (no scan published)",
          msg->header.frame_id.c_str(), target_frame_.c_str(), ex2.what());
        return;
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_out;
    tf2::doTransform(filtered, cloud_out, tf);
    cloud_out.header.stamp = msg->header.stamp;
    cloud_pub_->publish(cloud_out);

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "published %zu points in frame '%s'", points.size(), cloud_out.header.frame_id.c_str());
  }

  std::string target_frame_, input_topic_, output_topic_;
  std::string odom_topic_;
  double min_range_, max_range_, tf_timeout_, max_tf_age_;
  rclcpp::Time newest_state_stamp_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace ego_ground_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ego_ground_sim::PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
