#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <ego_planner/msg/bspline.hpp>
#include <ego_planner/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <plan_manage/inflation_escape.hpp>
#include <traj_utils/planning_visualization.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      ADJUST_POSE,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      ESCAPE_INFLATION
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    ego_planner::msg::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    double max_tracking_error_{-1.0};
    bool forward_only_{false}, inflation_escape_{false}, escape_target_valid_{false};
    double escape_speed_{0.06}, escape_hard_radius_{0.21};
    EscapePoints escape_obstacles_;
    Eigen::Vector2d escape_target_;
    rclcpp::Time escape_cloud_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time inflation_since_{0, 0, RCL_ROS_TIME};
    std::string planning_frame_;
    void escapeInflation();

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    FSM_EXEC_STATE exec_state_, last_state_;
    int continously_called_times_{0};

    /* motion data */
    geometry_msgs::msg::Twist cmd_vel;
    std_msgs::msg::UInt8 is_adjust_pose;
    double w_adjust;
    tf2::Quaternion quat;
    bool is_pos_update;
    bool is_target_receive;
    enum DIRECTION { POSITIVE = 0, NEGATIVE = 1 };
    DIRECTION dir;

    double roll, pitch, yaw, yaw_start, yaw_error_threshold, yaw_error;
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;
    bool odom_twist_in_body_frame_{false};

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    Eigen::Vector3d goal_last;
    int current_wp_;

    bool flag_escape_emergency_;
    bool preset_wps_pending_{false};

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr escape_cloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr escape_marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<ego_planner::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<ego_planner::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr adjust_cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_adjust_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr dir_pub, stop_pub;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void changeDirection();
    void checkYawError();
    double calculateYawError(double yaw_cur, double yaw_target);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    void getLocalTarget();
    void publishBspline();

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);

    bool checkCollision();

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(const rclcpp::Node::SharedPtr &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif
