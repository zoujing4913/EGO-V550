// Real sensor data are transformed numerically; no frame relabeling or fake TF.
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class SensorBridge : public rclcpp::Node
{
public:
  SensorBridge() : Node("ego_sensor_bridge")
  {
    frame_ = declare_parameter<std::string>("planning_frame", "odom_combined");
    base_ = declare_parameter<std::string>("base_frame", "base_footprint");
    const auto mode = declare_parameter<std::string>("sensor_mode", "scan");
    const auto input = declare_parameter<std::string>("input_topic", "/scan");
    min_range_ = declare_parameter<double>("min_range", 0.15);
    max_range_ = declare_parameter<double>("max_range", 8.0);
    self_radius_ = declare_parameter<double>("self_filter_radius", 0.0);
    min_height_ = declare_parameter<double>("min_height", 0.02);
    max_height_ = declare_parameter<double>("max_height", 3.0);
    max_age_ = declare_parameter<double>("max_sensor_age", 0.5);
    if (!(min_range_ >= 0 && max_range_ > min_range_ && self_radius_ >= 0 &&
      max_height_ > min_height_ && max_age_ > 0))
      throw std::invalid_argument("Invalid sensor range/height parameters");
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // EKF TF often arrives just after the LiDAR sample. A separate TF thread
    // lets lookupTransform wait for that exact stamp without blocking TF input.
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this, true);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ego/obstacles", 5);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ego/goal", 5);
    if (mode == "scan") {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(input,
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          if (!fresh(msg->header)) return;
          if (!std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_increment)) return;
          std::vector<geometry_msgs::msg::Point> points;
          for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const double r = msg->ranges[i];
            if (!std::isfinite(r) || r < std::max(min_range_, double(msg->range_min)) ||
              r > std::min(max_range_, double(msg->range_max))) continue;
            const double a = msg->angle_min + i * msg->angle_increment;
            geometry_msgs::msg::Point p;
            p.x = r * std::cos(a); p.y = r * std::sin(a);
            points.push_back(p);
          }
          publish(points, msg->header);
        });
    } else if (mode == "cloud") {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input,
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          if (!fresh(msg->header)) return;
          std::vector<geometry_msgs::msg::Point> points;
          try {
            sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
            for (; x != x.end(); ++x, ++y, ++z) {
              if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
              geometry_msgs::msg::Point p; p.x = *x; p.y = *y; p.z = *z;
              points.push_back(p);
            }
          } catch (const std::exception & e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid XYZ cloud: %s", e.what());
            return;
          }
          publish(points, msg->header);
        });
    } else throw std::invalid_argument("sensor_mode must be scan or cloud");
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 5,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (msg->header.frame_id.empty() || !std::isfinite(msg->pose.position.x) ||
          !std::isfinite(msg->pose.position.y)) return;
        try {
          auto result = *msg;
          if (msg->header.frame_id != frame_) {
            // RViz goals describe a destination, so use the current transform.
            auto tf = buffer_->lookupTransform(frame_, msg->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*msg, result, tf);
          }
          result.header.frame_id = frame_; result.header.stamp = now();
          goal_pub_->publish(result);
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN(get_logger(), "Goal rejected: %s", e.what());
        }
      });
  }

private:
  bool fresh(const std_msgs::msg::Header & header)
  {
    const double age = (now() - rclcpp::Time(header.stamp)).seconds();
    return !header.frame_id.empty() && rclcpp::Time(header.stamp).nanoseconds() > 0 &&
      age >= -0.05 && age <= max_age_;
  }

  void publish(const std::vector<geometry_msgs::msg::Point> & points,
    const std_msgs::msg::Header & header)
  {
    try {
      // Transform at the measurement time. Stale/latest TF is never substituted.
      const auto wait = rclcpp::Duration::from_seconds(0.15);
      const auto to_base = buffer_->lookupTransform(base_, header.frame_id, header.stamp, wait);
      const auto to_world = buffer_->lookupTransform(frame_, base_, header.stamp, wait);
      std::vector<geometry_msgs::msg::Point> kept;
      kept.reserve(points.size());
      for (const auto & p : points) {
        geometry_msgs::msg::PointStamped src, body, world;
        src.header = header; src.point = p;
        tf2::doTransform(src, body, to_base);
        const double r = std::hypot(body.point.x, body.point.y);
        if (!std::isfinite(r) || !std::isfinite(body.point.z) ||
          r < std::max(min_range_, self_radius_) || r > max_range_ || body.point.z < min_height_ ||
          body.point.z > max_height_) continue;
        tf2::doTransform(body, world, to_world);
        kept.push_back(world.point);
      }
      sensor_msgs::msg::PointCloud2 out;
      out.header = header; out.header.frame_id = frame_;
      sensor_msgs::PointCloud2Modifier mod(out);
      mod.setPointCloud2FieldsByString(1, "xyz"); mod.resize(kept.size());
      sensor_msgs::PointCloud2Iterator<float> x(out, "x"), y(out, "y"), z(out, "z");
      for (const auto & p : kept) { *x = p.x; *y = p.y; *z = p.z; ++x; ++y; ++z; }
      cloud_pub_->publish(out);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Obstacle cloud dropped: %s", e.what());
    }
  }
  std::string frame_, base_;
  double min_range_, max_range_, self_radius_, min_height_, max_height_, max_age_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorBridge>());
  rclcpp::shutdown();
}
