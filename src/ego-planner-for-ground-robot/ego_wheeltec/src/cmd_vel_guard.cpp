// Single real-chassis command publisher. Restart/lost perception disarms it.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

class CmdVelGuard : public rclcpp::Node
{
  using Clock = std::chrono::steady_clock;
public:
  CmdVelGuard() : Node("ego_cmd_vel_guard")
  {
    frame_ = declare_parameter<std::string>("planning_frame", "odom_combined");
    base_ = declare_parameter<std::string>("base_frame", "base_footprint");
    const auto odom = declare_parameter<std::string>("odom_topic", "/odom_combined");
    const auto output = declare_parameter<std::string>("output_topic", "/ego/cmd_vel_checked");
    max_v_ = declare_parameter<double>("max_linear", 0.15);
    max_w_ = declare_parameter<double>("max_angular", 0.4);
    timeout_ = declare_parameter<double>("data_timeout", 0.7);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.25);
    obstacle_guard_ = declare_parameter<bool>("obstacle_guard", false);
    require_navigation_ = declare_parameter<bool>("require_navigation_ready", false);
    forward_only_ = declare_parameter<bool>("forward_only", false);
    radius_ = declare_parameter<double>("robot_radius", 0.30);
    margin_ = declare_parameter<double>("collision_margin", 0.10);
    reaction_ = declare_parameter<double>("reaction_time", 0.5);
    deceleration_ = declare_parameter<double>("braking_deceleration", 0.2);
    if (!(std::isfinite(max_v_) && max_v_ > 0 && std::isfinite(max_w_) && max_w_ > 0 &&
      std::isfinite(timeout_) && timeout_ > 0 && cmd_timeout_ > 0 &&
      std::isfinite(radius_) && radius_ > 0 && std::isfinite(margin_) && margin_ >= 0 &&
      std::isfinite(reaction_) && reaction_ > 0 && std::isfinite(deceleration_) && deceleration_ > 0))
      throw std::invalid_argument("Invalid watchdog limits/timeouts");
    out_ = create_publisher<geometry_msgs::msg::Twist>(output, 1);
    status_ = create_publisher<std_msgs::msg::String>("/ego/guard_state", 1);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const auto & p = m->pose.pose.position; const auto & q = m->pose.pose.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        const auto & v = m->twist.twist.linear;
        if (m->header.frame_id == frame_ && m->child_frame_id == base_ && fresh(m->header) &&
          std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
          std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
          std::isfinite(norm) && std::abs(norm - 1.0) < 0.05 &&
          std::isfinite(m->twist.twist.angular.z)) {odom_ = *m; odom_time_ = Clock::now();}
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/ego/obstacles",
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        if (m->header.frame_id != frame_ || !fresh(m->header)) return;
        if (obstacle_guard_) {
          try {
            std::vector<std::pair<double, double>> points;
            sensor_msgs::PointCloud2ConstIterator<float> x(*m, "x"), y(*m, "y");
            for (; x != x.end(); ++x, ++y) {
              if (!std::isfinite(*x) || !std::isfinite(*y)) return;
              points.emplace_back(*x, *y);
            }
            obstacles_ = std::move(points);
          } catch (const std::runtime_error &) {return;}
        }
        cloud_time_ = Clock::now();
      });
    navigation_sub_ = create_subscription<std_msgs::msg::Bool>("/ego/navigation_ready", 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        navigation_ready_ = m->data; navigation_time_ = Clock::now();
      });
    exploration_sub_ = create_subscription<geometry_msgs::msg::Twist>("/ego/exploration_turn", 1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        if (m->linear.x == 0 && m->linear.y == 0 && m->linear.z == 0 &&
          m->angular.x == 0 && m->angular.y == 0 && std::isfinite(m->angular.z)) {
          exploration_turn_ = m->angular.z; exploration_time_ = Clock::now();
        }
      });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map_2d", 1,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
        if (m->header.frame_id == frame_ && fresh(m->header) && m->info.width > 0 &&
          m->info.height > 0 && m->data.size() == size_t(m->info.width) * m->info.height)
          map_time_ = Clock::now();
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("/ego/cmd_vel_raw", 1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        const bool valid = std::isfinite(m->linear.x) && std::isfinite(m->angular.z) &&
          m->linear.y == 0 && m->linear.z == 0 && m->angular.x == 0 && m->angular.y == 0;
        if (!valid) { enabled_ = false; reason_ = "invalid command; re-enable and send a new goal"; return; }
        if (goal_after_enable_) { command_ = *m; cmd_time_ = Clock::now(); }
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/ego/goal", 5,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        if (enabled_ && m->header.frame_id == frame_ && fresh(m->header)) {
          goal_after_enable_ = true; command_ = geometry_msgs::msg::Twist(); cmd_time_ = {};
        }
      });
    service_ = create_service<std_srvs::srv::SetBool>("/ego/set_enabled",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
        std_srvs::srv::SetBool::Response::SharedPtr res) {
        command_ = geometry_msgs::msg::Twist(); goal_after_enable_ = false; cmd_time_ = {};
        const auto issue = inputIssue();
        enabled_ = req->data && issue.empty();
        reason_ = req->data ? (enabled_ ? "enabled; waiting for a NEW goal" : issue) : "disabled";
        res->success = !req->data || enabled_; res->message = reason_;
        out_->publish(geometry_msgs::msg::Twist());
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Output %s, disabled on startup; enable service /ego/set_enabled",
      output.c_str());
  }
private:
  bool fresh(const std_msgs::msg::Header & h) const
  {
    const double age = (now() - rclcpp::Time(h.stamp)).seconds();
    return rclcpp::Time(h.stamp).nanoseconds() > 0 && age >= -0.05 && age <= timeout_;
  }
  double age(Clock::time_point t) const
  { return t == Clock::time_point{} ? 1e9 : std::chrono::duration<double>(Clock::now()-t).count(); }
  std::string inputIssue() const
  {
    if (age(odom_time_) > timeout_) return "odometry missing, stale, invalid, or wrong frame";
    if (age(cloud_time_) > timeout_) return "obstacle cloud/TF missing or stale";
    if (age(map_time_) > timeout_) return "planner map missing or stale";
    if (require_navigation_ && age(navigation_time_) > timeout_) return "navigation heartbeat missing/stale";
    return "";
  }
  bool blocked(double speed, double turn) const
  {
    const auto & q = odom_.pose.pose.orientation;
    const double yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
    const double horizon = reaction_ + std::abs(speed)/deceleration_;
    const int steps = std::max(1, int(std::ceil(horizon/0.02)));
    const double clearance = radius_ + margin_ + std::abs(speed)*0.02;
    for (int i = 0; i <= steps; ++i) {
      const double t = horizon*i/steps;
      const double dx = std::abs(turn) < 1e-6 ? speed*t : speed*std::sin(turn*t)/turn;
      const double dy = std::abs(turn) < 1e-6 ? 0.0 : speed*(1-std::cos(turn*t))/turn;
      const double x = odom_.pose.pose.position.x + std::cos(yaw)*dx-std::sin(yaw)*dy;
      const double y = odom_.pose.pose.position.y + std::sin(yaw)*dx+std::cos(yaw)*dy;
      for (const auto & p : obstacles_) if (std::hypot(p.first-x, p.second-y) <= clearance) return true;
    }
    return false;
  }
  void tick()
  {
    geometry_msgs::msg::Twist out;
    const auto issue = inputIssue();
    if (enabled_ && !issue.empty()) { enabled_ = false; goal_after_enable_ = false; reason_ = issue; }
    if (enabled_ && goal_after_enable_ && age(cmd_time_) > cmd_timeout_) {
      // Allow time for the FIRST trajectory after a new goal; commands stay zero.
      if (cmd_time_ != Clock::time_point{}) {
        enabled_ = false; reason_ = "planner command timeout; re-enable and send a new goal";
      }
    } else if (enabled_ && goal_after_enable_) {
      out.linear.x = std::clamp(command_.linear.x, forward_only_ ? 0.0 : -max_v_, max_v_);
      out.angular.z = std::clamp(command_.angular.z, -max_w_, max_w_);
    }
    std::string pause;
    if (enabled_ && require_navigation_ && !navigation_ready_) pause = "waiting for an observed free route";
    if (obstacle_guard_ && (blocked(out.linear.x, out.angular.z) ||
      blocked(odom_.twist.twist.linear.x, odom_.twist.twist.angular.z))) pause = "obstacle in stopping envelope";
    if (!pause.empty()) out = geometry_msgs::msg::Twist();
    // Directional observation may rotate a stationary bounded footprint, but
    // can never translate into unknown space or bypass the measured envelope.
    if (enabled_ && goal_after_enable_ && require_navigation_ && !navigation_ready_ &&
      age(exploration_time_) < cmd_timeout_ && std::abs(odom_.twist.twist.linear.x) < 0.02 &&
      !blocked(0.0, exploration_turn_)) {
      out.angular.z = std::clamp(exploration_turn_, -max_w_, max_w_);
    }
    out_->publish(out);
    std_msgs::msg::String state;
    state.data = enabled_ ? (goal_after_enable_ ? "enabled; tracking" : "enabled; waiting for a NEW goal") : reason_;
    if (enabled_ && !pause.empty()) state.data = "enabled; " + pause;
    status_->publish(state);
  }
  std::string frame_, base_, reason_{"disabled on startup"};
  double max_v_, max_w_, timeout_, cmd_timeout_;
  double radius_, margin_, reaction_, deceleration_;
  double exploration_turn_{0.0};
  bool obstacle_guard_, require_navigation_, forward_only_, navigation_ready_{false};
  nav_msgs::msg::Odometry odom_;
  std::vector<std::pair<double, double>> obstacles_;
  bool enabled_{false}, goal_after_enable_{false};
  geometry_msgs::msg::Twist command_;
  Clock::time_point odom_time_{}, cloud_time_{}, map_time_{}, cmd_time_{};
  Clock::time_point navigation_time_{}, exploration_time_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr out_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr exploration_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelGuard>());
  rclcpp::shutdown();
}
