// Simulated chassis command watchdog. Optional LaserScan guard checks the
// commanded curved motion and braking envelope against measurement-time TF
// points, compensated using current odometry. V550 launches enable this guard;
// other existing simulation launches retain the timeout-only default.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "ego_ground_sim/scan_guard.hpp"
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ego_ground_sim
{

class CmdVelSafety : public rclcpp::Node
{
public:
  CmdVelSafety()
  : rclcpp::Node("cmd_vel_safety")
  {
    cmd_timeout_ = this->declare_parameter<double>("cmd_timeout", 0.5);
    odom_timeout_ = this->declare_parameter<double>("odom_timeout", 1.0);
    max_linear_ = this->declare_parameter<double>("max_linear", 2.0);
    max_angular_ = this->declare_parameter<double>("max_angular", 2.5);
    const double rate = this->declare_parameter<double>("publish_rate", 50.0);
    use_scan_guard_ = declare_parameter<bool>("use_scan_guard", false);
    forward_only_ = declare_parameter<bool>("forward_only", false);
    radius_ = declare_parameter<double>("robot_radius", 0.18);
    hard_margin_ = declare_parameter<double>("hard_margin", 0.03);
    margin_ = declare_parameter<double>("collision_margin", 0.12);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.35);
    reaction_time_ = declare_parameter<double>("reaction_time", 0.8);
    deceleration_ = declare_parameter<double>("braking_deceleration", 0.4);
    frame_ = declare_parameter<std::string>("planning_frame", "odom_combined");
    if (!(radius_ > 0 && margin_ >= 0 && hard_margin_ >= 0 && hard_margin_ <= margin_ && scan_timeout_ > 0 &&
      reaction_time_ > 0 && deceleration_ > 0)) {
      throw std::invalid_argument("Invalid scan collision guard parameters");
    }

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&CmdVelSafety::cmdCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10, std::bind(&CmdVelSafety::odomCallback, this, std::placeholders::_1));

    last_cmd_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_odom_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_scan_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    if (use_scan_guard_) {
      buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this, true);
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan",
        rclcpp::SensorDataQoS(), std::bind(&CmdVelSafety::scanCallback, this, std::placeholders::_1));
      collision_pub_ = create_publisher<std_msgs::msg::Bool>("/collision_guard/stopped", 10);
    }

    // The node clock is used here on purpose: the "age" of the data has to be
    // measured on the same clock that timestamps the simulation.
    timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
      std::bind(&CmdVelSafety::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(), "watchdog: cmd timeout %.2fs, odom timeout %.2fs, limits v=%.2f w=%.2f",
      cmd_timeout_, odom_timeout_, max_linear_, max_angular_);
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    cmd_ = *msg;
    last_cmd_ = this->now();
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (use_scan_guard_ && msg->header.frame_id != frame_) {return;}
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(q.x) ||
      !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
      q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < 1e-6 ||
      !std::isfinite(msg->twist.twist.linear.x) || !std::isfinite(msg->twist.twist.angular.z)) {return;}
    odom_ = *msg;
    last_odom_ = msg->header.stamp;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    const double age = (now()-rclcpp::Time(msg->header.stamp)).seconds();
    if (age < -0.05 || age > scan_timeout_ || msg->ranges.size() < 4 ||
      !std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_increment)) {return;}
    try {
      const auto tf = buffer_->lookupTransform(frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.05));
      std::vector<Point2> points;
      size_t usable = 0;
      for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const double range = msg->ranges[i];
        // Positive infinity is a valid no-return beam; NaNs are invalid data.
        if (std::isinf(range) && range > 0) {++usable; continue;}
        if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {continue;}
        ++usable;
        const double angle = msg->angle_min+i*msg->angle_increment;
        geometry_msgs::msg::PointStamped point, world;
        point.point.x = range*std::cos(angle); point.point.y = range*std::sin(angle);
        tf2::doTransform(point, world, tf);
        points.push_back({world.point.x, world.point.y});
      }
      if (usable == 0) {return;}
      obstacles_ = std::move(points);
      last_scan_ = msg->header.stamp;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "scan guard TF: %s", e.what());
    }
  }

  bool collisionBlocked(double speed, double turn)
  {
    const auto & q = odom_.pose.pose.orientation;
    const double yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
    const double c = std::cos(yaw), s = std::sin(yaw);
    std::vector<Point2> local;
    local.reserve(obstacles_.size());
    for (const auto & p : obstacles_) {
      const double dx = p.x-odom_.pose.pose.position.x, dy = p.y-odom_.pose.pose.position.y;
      local.push_back({c*dx+s*dy, -s*dx+c*dy});
    }
    // Include measured velocity if the chassis is still moving faster than the request.
    const double measured = odom_.twist.twist.linear.x;
    return guardMotionBlocked(local, speed, turn, radius_+hard_margin_, radius_+margin_, reaction_time_, deceleration_) ||
      (std::abs(measured) > std::abs(speed)+0.02 &&
      guardMotionBlocked(local, measured, odom_.twist.twist.angular.z,
        radius_+hard_margin_, radius_+margin_, reaction_time_, deceleration_));
  }

  void timerCallback()
  {
    const rclcpp::Time now = this->now();
    geometry_msgs::msg::Twist out;

    const double cmd_age = (now - last_cmd_).seconds();
    const double odom_age = (now - last_odom_).seconds();
    const bool cmd_fresh = last_cmd_.nanoseconds() > 0 && cmd_age >= 0 && cmd_age <= cmd_timeout_;
    const bool odom_fresh = last_odom_.nanoseconds() > 0 && odom_age >= -0.05 && odom_age <= odom_timeout_;
    const bool cmd_valid = std::isfinite(cmd_.linear.x) && std::isfinite(cmd_.angular.z);

    if (!cmd_fresh) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "no /cmd_vel for %.2fs -> stopping", cmd_age);
    } else if (!odom_fresh) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "no /state_estimation for %.2fs -> stopping", odom_age);
    } else if (!cmd_valid) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "invalid (NaN/Inf) velocity command -> stopping");
    } else {
      out.linear.x = std::clamp(cmd_.linear.x, forward_only_ ? 0.0 : -max_linear_, max_linear_);
      out.angular.z = std::clamp(cmd_.angular.z, -max_angular_, max_angular_);
    }

    if (use_scan_guard_) {
      const double age = (now-last_scan_).seconds();
      const bool scan_fresh = last_scan_.nanoseconds() > 0 && age >= -0.05 && age <= scan_timeout_;
      const bool blocked = !scan_fresh || !odom_fresh || collisionBlocked(out.linear.x, out.angular.z);
      std_msgs::msg::Bool status; status.data = blocked;
      collision_pub_->publish(status);
      if (blocked) {
        if (std::abs(out.linear.x) > 0.001 || std::abs(out.angular.z) > 0.001) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "scan guard STOP: %s", scan_fresh ? "obstacle in chassis swept area" : "scan stale/missing");
        }
        if (scan_fresh && odom_fresh && cmd_fresh && cmd_valid &&
          !collisionBlocked(0.0, out.angular.z)) {out.linear.x = 0.0;}
        else {out = geometry_msgs::msg::Twist();}
      }
    }

    cmd_pub_->publish(out);
  }

  double cmd_timeout_, odom_timeout_, max_linear_, max_angular_;
  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time last_cmd_, last_odom_;
  bool use_scan_guard_, forward_only_;
  double radius_, margin_, hard_margin_, scan_timeout_, reaction_time_, deceleration_;
  std::string frame_;
  rclcpp::Time last_scan_;
  nav_msgs::msg::Odometry odom_;
  std::vector<Point2> obstacles_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ego_ground_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ego_ground_sim::CmdVelSafety>());
  rclcpp::shutdown();
  return 0;
}
