#include <algorithm>
#include <cmath>
#include <chrono>
#include <bspline_opt/uniform_bspline.h>
#include <ego_planner/msg/bspline.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>

// Visualize the same spline consumed by MPC, rather than its control polygon.
// A small display-only Z offset keeps the lines above the occupancy map.
class TrajectoryVisualizer : public rclcpp::Node
{
public:
  TrajectoryVisualizer() : Node("v550_trajectory_visualizer")
  {
    frame_ = declare_parameter<std::string>("frame_id", "odom_combined");
    planned_.header.frame_id = actual_.header.frame_id = frame_;
    auto qos = rclcpp::QoS(1).transient_local();
    planned_pub_ = create_publisher<nav_msgs::msg::Path>("/planning/bspline_path", qos);
    actual_pub_ = create_publisher<nav_msgs::msg::Path>("/v550_sim/actual_path", qos);
    spline_sub_ = create_subscription<ego_planner::msg::Bspline>(
      "/planning/bspline", 10, [this](const ego_planner::msg::Bspline &msg) {
        const int count = static_cast<int>(msg.pos_pts.size());
        if (msg.order < 1 || count <= msg.order ||
            msg.knots.size() != msg.pos_pts.size() + msg.order + 1 ||
            !std::is_sorted(msg.knots.begin(), msg.knots.end())) return;
        Eigen::MatrixXd points(3, count);
        for (int i = 0; i < count; ++i) {
          const auto &p = msg.pos_pts[i];
          points.col(i) = Eigen::Vector3d(p.x, p.y, p.z);
        }
        const auto knots = Eigen::Map<const Eigen::VectorXd>(msg.knots.data(), msg.knots.size());
        if (!points.allFinite() || !knots.allFinite()) return;
        ego_planner::UniformBspline spline(points, msg.order, 0.1);
        spline.setKnot(knots);
        const double duration = spline.getTimeSum();
        if (!std::isfinite(duration) || duration <= 0 || duration > 600) return;
        planned_.poses.clear();
        planned_.header.stamp = now();
        const int samples = std::max(2, static_cast<int>(std::ceil(duration / 0.03)));
        for (int i = 0; i <= samples; ++i) {
          const double t = duration * i / samples;
          const auto p = spline.evaluateDeBoorT(t);
          geometry_msgs::msg::PoseStamped pose;
          pose.header = planned_.header;
          pose.header.stamp = rclcpp::Time(msg.start_time) + rclcpp::Duration::from_seconds(t);
          pose.pose.position.x = p.x();
          pose.pose.position.y = p.y();
          pose.pose.position.z = 0.08;
          pose.pose.orientation.w = 1;
          planned_.poses.push_back(pose);
        }
        planned_pub_->publish(planned_);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_combined", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry &msg) {
        if (msg.header.frame_id != frame_) return;
        const auto &p = msg.pose.pose.position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) return;
        if (!actual_.poses.empty()) {
          const auto &last = actual_.poses.back().pose.position;
          if (std::hypot(p.x - last.x, p.y - last.y) < 0.02) return;
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose = msg.pose.pose;
        pose.pose.position.z = 0.11;
        actual_.poses.push_back(pose);
        if (actual_.poses.size() > 6000)
          actual_.poses.erase(actual_.poses.begin(), actual_.poses.begin() + 1000);
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ego/goal", 10, [this](const geometry_msgs::msg::PoseStamped &) {
        actual_.poses.clear();
        planned_.poses.clear();
        planned_.header.stamp = now();
        planned_pub_->publish(planned_);
      });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(200), [this]() {
      actual_.header.stamp = now();
      actual_pub_->publish(actual_);
      if (!planned_.poses.empty()) planned_pub_->publish(planned_);
    });
  }
private:
  std::string frame_;
  nav_msgs::msg::Path planned_, actual_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_pub_, actual_pub_;
  rclcpp::Subscription<ego_planner::msg::Bspline>::SharedPtr spline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryVisualizer>());
  rclcpp::shutdown();
}
