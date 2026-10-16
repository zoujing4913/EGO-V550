#include "bspline_opt/uniform_bspline.h"
#include "MPC.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ego_planner/msg/bspline.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <plan_env/param_utils.hpp>
#include <rclcpp/create_timer.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include "time.h"

#define PI 3.1415926
#define yaw_error_max 90.0/180*PI
int N = 15;

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub;

//quadrotor_msgs::PositionCommand cmd;
geometry_msgs::msg::Twist cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool odom_twist_in_body_frame_ = false;
bool forward_only_ = false;
bool is_orientation_init = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_{0, 0, RCL_ROS_TIME},time_s,time_e;
int traj_id_;

Eigen::Vector3d odom_pos_,odom_vel_;
Eigen::Quaterniond odom_orient_;

MPC_controller mpc_controller;
double roll,pitch,yaw;
geometry_msgs::msg::PoseStamped pose_cur;
tf2::Quaternion quat;
std_msgs::msg::UInt8 is_adjust_pose;
std_msgs::msg::UInt8 dir;

enum DIRECTION {POSITIVE=0,NEGATIVE=1};

// yaw control
double t_step;

std_msgs::msg::UInt8 stop_command;

////time record
clock_t start_clock,end_clock;
double duration;

void bsplineCallback(const ego_planner::msg::Bspline::ConstSharedPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);


  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  //ROS_INFO("Receive b-spline trajectory!");

  receive_traj_ = true;

}

void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    pose_cur = *msg;
    quat = tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                           msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}

void adjust_yaw_Callback(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    is_adjust_pose = *msg;
}

void dirCallback(const std_msgs::msg::UInt8::ConstSharedPtr& msg)
{
    dir = *msg;
    if (forward_only_) {dir.data = POSITIVE;}
}

// X_r contains states after each prediction step, U_r the input at its start.
// Both are sampled from the exact B-spline received on /planning/bspline.
void MPC_calculate(double &t_cur)
{
    std::vector<Eigen::Vector3d> X_r;
    std::vector<Eigen::Vector2d> U_r;
    auto heading = [](double t) {
        // The terminal derivative is zero; use its incoming tangent there.
        const double sample = std::clamp(t, 0.0, std::max(0.0, traj_duration_ - 0.02));
        const Eigen::Vector3d velocity = traj_[1].evaluateDeBoorT(sample);
        return velocity.head<2>().norm() > 1e-6 ? std::atan2(velocity.y(), velocity.x()) : yaw;
    };
    double previous_yaw = yaw;
    for (int i = 0; i < N; ++i)
    {
        const double t0 = std::min(t_cur + i*t_step, traj_duration_);
        const double t1 = std::min(t_cur + (i+1)*t_step, traj_duration_);
        const Eigen::Vector3d position = traj_[0].evaluateDeBoorT(t1);
        const Eigen::Vector3d velocity = traj_[1].evaluateDeBoorT(t0);
        const double yaw0 = heading(t0);
        const double yaw1 = previous_yaw + std::remainder(heading(t1)-previous_yaw, 2*M_PI);
        const double turn_rate = t0 < traj_duration_ ?
            std::remainder(heading(t1)-yaw0, 2*M_PI)/t_step : 0.0;
        X_r.emplace_back(position.x(), position.y(), yaw1);
        U_r.emplace_back(velocity.head<2>().norm(), turn_rate);
        previous_yaw = yaw1;
    }
    const Eigen::Vector3d current(odom_pos_.x(), odom_pos_.y(), yaw);
    const Eigen::MatrixXd inputs = mpc_controller.MPC_Solve_qp(current, X_r, U_r, N);
    cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = (dir.data == NEGATIVE ? -1.0 : 1.0)*inputs(0,0);
    cmd.angular.z = inputs(1,0);
}

void stopCallback(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    stop_command = *msg;
}

void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    if (odom_twist_in_body_frame_)
        odom_vel_ = odom_orient_.normalized() * odom_vel_;

    quat = tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if(dir.data==NEGATIVE)
    {
        if(yaw>0)
        {
            yaw -= PI;
        }else if(yaw<0)
        {
            yaw += PI;
        }
    }

}

void cmdCallback()
{
    /* no publishing before receive traj_ */
    if (stop_command.data==1)
    {
        cmd.angular.z = 0;
        cmd.linear.x = 0;
        vel_cmd_pub->publish(cmd);
        return;
    }

    // The FSM owns /cmd_vel while rotating in place. Publishing an old MPC
    // trajectory (including zero commands) here would overwrite its yaw command.
    if (is_adjust_pose.data == 1)
        return;

    if (!receive_traj_)
        return;
    //ROS_WARN("Run here !");
    rclcpp::Time time_s = node_->now();
    double t_cur = (time_s - start_time_).seconds();

//    Eigen::Vector3d pos_first = traj_[0].evaluateDeBoor(t_cur);
//    Eigen::Vector3d pos_second = traj_[0].evaluateDeBoor(t_cur+t_step);
//    double yaw_start = atan2((pos_second-pos_first)(1),(pos_second-pos_first)(0));

    static rclcpp::Time time_last = node_->now();

    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        //ROS_INFO("MPC_Calculate!");
        start_clock = clock();
        MPC_calculate(t_cur);
        end_clock = clock();
        duration = (double)(end_clock - start_clock) / CLOCKS_PER_SEC *1000;
        //ROS_INFO("Control times : %f ms",duration);
    }
    else if (t_cur >= traj_duration_)
    {
        cmd.angular.z = 0;
        cmd.linear.x = 0;
        vel_cmd_pub->publish(cmd);
        is_orientation_init=false;
    }
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }
    time_last = time_s;

    vel_cmd_pub->publish(cmd);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node_ = std::make_shared<rclcpp::Node>("traj_server");
  odom_twist_in_body_frame_ = plan_env::getParam<bool>(node_, "odom_twist_in_body_frame", false);
  forward_only_ = plan_env::getParam<bool>(node_, "forward_only", false);
  N = std::clamp(plan_env::getParam<int>(node_, "MPC.horizon_steps", 15), 2, 60);

  // Optional pose source (used to know the current yaw). Left empty by default so
  // that the odometry topic is the only attitude source, as in the ROS 1 version.
  std::string pose_topic = plan_env::getParam<std::string>(node_, "pose_topic", std::string(""));

  auto bspline_sub = node_->create_subscription<ego_planner::msg::Bspline>(
      "/planning/bspline", 10, bsplineCallback);
  if (!pose_topic.empty())
  {
    auto pose_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, 10, poseCallback);
  }
  auto odom_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10, odometryCallback);
  auto stop_sub = node_->create_subscription<std_msgs::msg::UInt8>(
      "/emergency_stop", 10, stopCallback);
  auto adjust_yaw_sub = node_->create_subscription<std_msgs::msg::UInt8>(
      "/is_adjust_yaw", 10, adjust_yaw_Callback);
  auto command_sub = node_->create_subscription<std_msgs::msg::UInt8>(
      "/direction", 10, dirCallback);

  mpc_controller.MPC_init(node_);
  vel_cmd_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 50);
  stop_command.data = 0;
  dir.data = POSITIVE;
  t_step = 0.03;


  // node clock timer -> follows simulation time when use_sim_time is enabled
  auto cmd_timer = rclcpp::create_timer(node_, node_->get_clock(), std::chrono::milliseconds(30),
                                        std::function<void()>(cmdCallback));

//  nh.param("traj_server/time_forward", time_forward_, -1.0);
//  last_yaw_ = 0.0;
//  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node_->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node_);
  rclcpp::shutdown();

  return 0;
}
